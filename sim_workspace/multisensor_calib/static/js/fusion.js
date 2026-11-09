// 全局变量
let topics = [];
let subscribedTopics = new Set();
let pollingTimer = null;

// Three.js 变量
let scene, camera, renderer, controls;
let pointCloudObj = null;

// 初始化
document.addEventListener('DOMContentLoaded', function() {
    initThree();
    loadTopics();
    startPolling();
});

// 离开页面时关闭所有融合订阅
window.addEventListener('beforeunload', function() {
    navigator.sendBeacon('/api/fusion/unsubscribe_all');
});

// 点击"返回标定页"时关闭融合订阅再跳转
document.addEventListener('DOMContentLoaded', function() {
    const backLink = document.querySelector('.back-link');
    if (backLink) {
        backLink.addEventListener('click', function(e) {
            e.preventDefault();
            fetch('/api/fusion/unsubscribe_all', { method: 'POST' })
                .finally(() => {
                    window.location.href = '/';
                });
        });
    }
});

// ========= Three.js 初始化 =========
function initThree() {
    const container = document.getElementById('threeContainer');
    const width = container.clientWidth;
    const height = container.clientHeight;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);

    camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
    // ROS 坐标系：x 前、y 左、z 上。相机从右前上方看
    camera.up.set(0, 0, 1);
    camera.position.set(15, -15, 12);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    // OrbitControls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.minDistance = 2;
    controls.maxDistance = 100;

    // 自定义坐标轴：x(红, 前)、y(绿, 左)、z(蓝, 上)
    const axisLength = 8;
    function makeAxis(dir, color) {
        const geom = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(dir[0]*axisLength, dir[1]*axisLength, dir[2]*axisLength)
        ]);
        return new THREE.Line(geom, new THREE.LineBasicMaterial({ color: color, linewidth: 2 }));
    }
    scene.add(makeAxis([1, 0, 0], 0xff0000));  // x 红 (前)
    scene.add(makeAxis([0, 1, 0], 0x00ff00));  // y 绿 (左)
    scene.add(makeAxis([0, 0, 1], 0x0000ff));  // z 蓝 (上)

    // 网格平面在 XY 平面（Z=0），符合 ROS 地面
    const grid = new THREE.GridHelper(20, 20, 0x444444, 0x333333);
    grid.rotation.x = Math.PI / 2;  // 旋转到 XY 平面
    scene.add(grid);

    // 灯光
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambient);

    window.addEventListener('resize', onResize);
    animate();
}

function onResize() {
    const container = document.getElementById('threeContainer');
    const width = container.clientWidth;
    const height = container.clientHeight;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// ========= 话题列表 =========
function loadTopics() {
    fetch('/api/topics')
        .then(r => r.json())
        .then(data => {
            topics = data.topics.filter(t => t.type === 'sensor_msgs/PointCloud2');
            renderTopicList();
        })
        .catch(err => console.error('Failed to load topics:', err));
}

function renderTopicList() {
    const list = document.getElementById('topicList');
    const filter = document.getElementById('topicFilter').value.toLowerCase();
    list.innerHTML = '';

    topics
        .filter(t => t.name.toLowerCase().includes(filter))
        .forEach(topic => {
            const li = document.createElement('li');
            li.className = 'topic-item';
            if (subscribedTopics.has(topic.name)) {
                li.classList.add('subscribed');
            }
            li.textContent = topic.name;
            li.addEventListener('click', () => toggleSubscribe(topic));
            list.appendChild(li);
        });
}

document.getElementById('topicFilter').addEventListener('input', renderTopicList);

// 订阅/取消订阅
function toggleSubscribe(topic) {
    if (subscribedTopics.has(topic.name)) {
        // 取消订阅
        fetch('/api/fusion/unsubscribe', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ topic: topic.name })
        })
        .then(() => {
            subscribedTopics.delete(topic.name);
            renderTopicList();
            updateSubCount();
        });
    } else {
        // 订阅
        fetch('/api/fusion/subscribe', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ topic: topic.name })
        })
        .then(() => {
            subscribedTopics.add(topic.name);
            renderTopicList();
            updateSubCount();
        });
    }
}

function updateSubCount() {
    document.getElementById('subCount').textContent = subscribedTopics.size;
}

// ========= 轮询获取融合点云 =========
function startPolling() {
    function tick() {
        fetch('/api/fusion/pointcloud?t=' + Date.now())
            .then(r => r.json())
            .then(data => {
                updatePointCloud(data);
            })
            .catch(err => console.error('Poll error:', err));
    }
    tick();
    pollingTimer = setInterval(tick, 500);
}

function updatePointCloud(data) {
    document.getElementById('pointCount').textContent = data.num_points;

    // 移除旧点云
    if (pointCloudObj) {
        scene.remove(pointCloudObj);
        pointCloudObj.geometry.dispose();
        pointCloudObj.material.dispose();
        pointCloudObj = null;
    }

    if (data.num_points === 0) return;

    // 从扁平数组重建 Float32Array
    const positions = new Float32Array(data.points);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    const material = new THREE.PointsMaterial({
        color: 0x00ff00,
        size: 0.05,
        sizeAttenuation: true
    });

    pointCloudObj = new THREE.Points(geometry, material);
    scene.add(pointCloudObj);
}
