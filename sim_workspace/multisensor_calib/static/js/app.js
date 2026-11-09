// 全局变量
let topics = [];
let subscribedTopics = new Set();
let currentLidarTopic = null;
let pollingTimer = null;
let currentPollingType = null; // 'image' | 'pointcloud' | 'message'

// Canvas 上下文
let xyCanvas, xzCanvas, yzCanvas;
let xyCtx, xzCtx, yzCtx;

// 投影线配置
let projectionLines = {
    xy_h: 0, xy_v: 0,
    xz_h: 0, xz_v: 0,
    yz_h: 0, yz_v: 0
};

// 点击融合页面链接时关闭标定页订阅
document.addEventListener('DOMContentLoaded', function() {
    const fusionLink = document.querySelector('.fusion-link');
    if (fusionLink) {
        fusionLink.addEventListener('click', function(e) {
            e.preventDefault();
            // 关闭标定页订阅
            fetch('/api/unsubscribe_all', { method: 'POST' })
                .finally(() => {
                    window.location.href = '/fusion';
                });
        });
    }

    initCanvas();
    initEventListeners();
    loadTopics();
});

// ========= HTTP 轮询（完全不用 WebSocket）=========
function startPolling(type) {
    stopPolling();
    currentPollingType = type;
    if (type === 'image') {
        startImagePolling();
    } else if (type === 'pointcloud') {
        pollData('/api/latest_pointcloud', handlePointcloudResponse, 300);
    }
}

function stopPolling() {
    if (pollingTimer) {
        clearInterval(pollingTimer);
        pollingTimer = null;
    }
    currentPollingType = null;
}

function pollData(url, handler, intervalMs) {
    function tick() {
        fetch(url + '?t=' + Date.now())
            .then(r => r.ok ? r.json() : null)
            .then(data => { if (data) handler(data); })
            .catch(err => console.error('Poll error:', err));
    }
    tick();
    pollingTimer = setInterval(tick, intervalMs);
}

// 图像用 blob 方式轮询
function startImagePolling() {
    stopPolling();
    currentPollingType = 'image';
    function tick() {
        fetch('/api/latest_image?t=' + Date.now())
            .then(response => {
                if (!response.ok) return null;
                const topic = response.headers.get('X-Topic') || '';
                const width = response.headers.get('X-Width') || '';
                const height = response.headers.get('X-Height') || '';
                return response.blob().then(blob => ({ blob, topic, width, height }));
            })
            .then(data => {
                if (!data) return;
                const url = URL.createObjectURL(data.blob);
                const img = document.getElementById('imageDisplay');
                img.onload = function() { URL.revokeObjectURL(url); };
                img.src = url;
                document.getElementById('imageInfo').textContent =
                    data.topic + ' | ' + data.width + 'x' + data.height;
            })
            .catch(err => console.error('Image poll error:', err));
    }
    tick();
    pollingTimer = setInterval(tick, 200);
}

function handlePointcloudResponse(data) {
    handlePointCloudUpdate(data);
}

// 初始化 Canvas
function initCanvas() {
    xyCanvas = document.getElementById('xyCanvas');
    xzCanvas = document.getElementById('xzCanvas');
    yzCanvas = document.getElementById('yzCanvas');

    xyCtx = xyCanvas.getContext('2d');
    xzCtx = xzCanvas.getContext('2d');
    yzCtx = yzCanvas.getContext('2d');

    // 根据容器大小动态设置 canvas 像素尺寸
    function resizeCanvases() {
        [xyCanvas, xzCanvas, yzCanvas].forEach(canvas => {
            const rect = canvas.getBoundingClientRect();
            canvas.width = Math.max(rect.width, 200);
            canvas.height = Math.max(rect.height, 200);
        });
    }
    resizeCanvases();
    window.addEventListener('resize', resizeCanvases);
}

// 初始化事件监听器
function initEventListeners() {
    // 话题过滤
    document.getElementById('topicFilter').addEventListener('input', filterTopics);
    document.getElementById('typeFilter').addEventListener('change', filterTopics);
    
    // Tab 切换
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.addEventListener('click', function() {
            const tabName = this.dataset.tab;
            switchTab(tabName);
        });
    });
    
    // 激光雷达选择
    document.getElementById('selectLidarBtn').addEventListener('click', selectLidar);

    // 保存参数
    document.getElementById('saveParamsBtn').addEventListener('click', saveParams);

    // 参数变化时自动应用
    const paramInputs = ['paramX', 'paramY', 'paramZ', 'paramRoll', 'paramPitch', 'paramYaw'];
    paramInputs.forEach(id => {
        document.getElementById(id).addEventListener('change', applyTransform);
    });

    // 投影线键盘控制
    initLineKeyboardControl();
}

// 加载话题列表
function loadTopics() {
    fetch('/api/topics')
        .then(response => response.json())
        .then(data => {
            // 只保留图像和点云话题
            topics = data.topics.filter(t =>
                t.type === 'sensor_msgs/PointCloud2' ||
                t.type === 'sensor_msgs/Image' ||
                t.type === 'sensor_msgs/CompressedImage'
            );
            renderTopicList();
            updateLidarSelect();
        })
        .catch(error => {
            console.error('Failed to load topics:', error);
            addLog('加载话题列表失败');
        });
}

// 渲染话题列表
function renderTopicList() {
    const list = document.getElementById('topicList');
    const filter = document.getElementById('topicFilter').value.toLowerCase();
    const typeFilter = document.getElementById('typeFilter').value;
    
    list.innerHTML = '';
    
    const filteredTopics = topics.filter(topic => {
        const matchName = topic.name.toLowerCase().includes(filter);
        let matchType = true;
        
        if (typeFilter === 'sensor_msgs/PointCloud2') {
            matchType = topic.type === 'sensor_msgs/PointCloud2';
        } else if (typeFilter === 'sensor_msgs/Image') {
            matchType = topic.type === 'sensor_msgs/Image';
        } else if (typeFilter === 'sensor_msgs/CompressedImage') {
            matchType = topic.type === 'sensor_msgs/CompressedImage';
        }
        
        return matchName && matchType;
    });
    
    filteredTopics.forEach(topic => {
        const li = document.createElement('li');
        li.className = 'topic-item';
        if (subscribedTopics.has(topic.name)) {
            li.classList.add('subscribed');
        }
        
        const icon = getTopicIcon(topic.type);
        li.innerHTML = `
            <span class="topic-icon">${icon}</span>
            <span class="topic-name">${topic.name}</span>
            <span class="topic-type">${topic.type}</span>
        `;
        
        li.addEventListener('click', () => subscribeTopic(topic));
        list.appendChild(li);
    });
}

// 获取话题图标
function getTopicIcon(type) {
    if (type === 'sensor_msgs/PointCloud2') return '🔷';
    if (type === 'sensor_msgs/Image' || type === 'sensor_msgs/CompressedImage') return '🖼️';
    return '📄';
}

// 过滤话题
function filterTopics() {
    renderTopicList();
}

// 订阅话题（同时只允许一个）
function subscribeTopic(topic) {
    // 如果已订阅同一个话题，跳过
    if (subscribedTopics.has(topic.name)) {
        addLog('话题已订阅: ' + topic.name);
        return;
    }
    
    fetch('/api/subscribe', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ topic: topic.name, type: topic.type })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 清除之前的订阅状态
            subscribedTopics.clear();
            subscribedTopics.add(topic.name);
            renderTopicList();
            addLog('已订阅: ' + topic.name);

            // 根据类型切换 tab 和启动对应的 HTTP 轮询（只支持图像和点云）
            if (topic.type === 'sensor_msgs/PointCloud2') {
                startPolling('pointcloud');
                switchTab('pointcloud');
            } else if (topic.type === 'sensor_msgs/Image' || topic.type === 'sensor_msgs/CompressedImage') {
                startPolling('image');
                switchTab('image');
            }
        }
    })
    .catch(error => {
        console.error('Failed to subscribe:', error);
        addLog('订阅失败: ' + topic.name);
    });
}

// 更新激光雷达选择下拉框
function updateLidarSelect() {
    const select = document.getElementById('lidarSelect');
    select.innerHTML = '<option value="">请选择...</option>';
    
    topics.filter(t => t.type === 'sensor_msgs/PointCloud2')
        .forEach(topic => {
            const option = document.createElement('option');
            option.value = topic.name;
            option.textContent = topic.name;
            select.appendChild(option);
        });
}

// 选择激光雷达
function selectLidar() {
    const select = document.getElementById('lidarSelect');
    const topic = select.value;
    
    if (!topic) {
        addLog('请选择激光雷达话题');
        return;
    }
    
    fetch('/api/select_lidar', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ topic: topic })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            currentLidarTopic = topic;
            document.getElementById('currentTopic').textContent = '当前话题: ' + topic;
            addLog('已选择激光雷达: ' + topic);

            // 清除之前的订阅状态，启动点云轮询和切换 tab
            subscribedTopics.clear();
            subscribedTopics.add(topic);
            renderTopicList();
            startPolling('pointcloud');
            switchTab('pointcloud');
        }
    })
    .catch(error => {
        console.error('Failed to select lidar:', error);
    });
}

// 保存参数（变换矩阵 T + 话题）
function saveParams() {
    if (!currentLidarTopic) {
        addLog('请先选择激光雷达话题');
        return;
    }
    const params = {
        topic: currentLidarTopic,
        x: parseFloat(document.getElementById('paramX').value),
        y: parseFloat(document.getElementById('paramY').value),
        z: parseFloat(document.getElementById('paramZ').value),
        roll: parseFloat(document.getElementById('paramRoll').value),
        pitch: parseFloat(document.getElementById('paramPitch').value),
        yaw: parseFloat(document.getElementById('paramYaw').value)
    };
    fetch('/api/save_params', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    })
    .then(r => r.json())
    .then(data => {
        if (data.success) {
            addLog('参数已保存: ' + currentLidarTopic);
        } else {
            addLog('保存失败: ' + (data.error || ''));
        }
    })
    .catch(err => {
        console.error('Save failed:', err);
        addLog('保存失败');
    });
}

// 应用变换
function applyTransform() {
    const params = {
        x: parseFloat(document.getElementById('paramX').value),
        y: parseFloat(document.getElementById('paramY').value),
        z: parseFloat(document.getElementById('paramZ').value),
        roll: parseFloat(document.getElementById('paramRoll').value),
        pitch: parseFloat(document.getElementById('paramPitch').value),
        yaw: parseFloat(document.getElementById('paramYaw').value)
    };
    
    fetch('/api/transform', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(params)
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 更新变换矩阵显示
            const matrix = data.matrix;
            const matrixStr = matrix.map(row => 
                '[' + row.map(v => v.toFixed(4).padStart(8)).join(', ') + ']'
            ).join('\n');
            document.getElementById('transformMatrix').textContent = matrixStr;
            addLog('变换已应用');
        }
    })
    .catch(error => {
        console.error('Failed to apply transform:', error);
    });
}

// 投影线键盘控制
let activeCanvasId = null;
const LINE_STEP = 0.2;

function initLineKeyboardControl() {
    // 点击 canvas 选中
    [xyCanvas, xzCanvas, yzCanvas].forEach(canvas => {
        canvas.addEventListener('click', function() {
            activeCanvasId = this.id;
            this.style.borderColor = '#00d9ff';
            [xyCanvas, xzCanvas, yzCanvas].forEach(c => {
                if (c !== this) c.style.borderColor = '#3a3a3a';
            });
        });
    });

    // 键盘控制
    document.addEventListener('keydown', function(e) {
        if (!activeCanvasId) return;
        // 只处理方向键
        if (![37,38,39,40].includes(e.keyCode)) return;
        e.preventDefault();

        if (activeCanvasId === 'xyCanvas') {
            if (e.keyCode === 38) projectionLines.xy_h += LINE_STEP;      // ↑
            else if (e.keyCode === 40) projectionLines.xy_h -= LINE_STEP; // ↓
            else if (e.keyCode === 39) projectionLines.xy_v += LINE_STEP; // →
            else if (e.keyCode === 37) projectionLines.xy_v -= LINE_STEP; // ←
        } else if (activeCanvasId === 'xzCanvas') {
            if (e.keyCode === 38) projectionLines.xz_h += LINE_STEP;
            else if (e.keyCode === 40) projectionLines.xz_h -= LINE_STEP;
        } else if (activeCanvasId === 'yzCanvas') {
            if (e.keyCode === 38) projectionLines.yz_h += LINE_STEP;
            else if (e.keyCode === 40) projectionLines.yz_h -= LINE_STEP;
        }

        // 更新显示
        document.getElementById('valXYH').textContent = projectionLines.xy_h.toFixed(1);
        document.getElementById('valXYV').textContent = projectionLines.xy_v.toFixed(1);
        document.getElementById('valXZH').textContent = projectionLines.xz_h.toFixed(1);
        document.getElementById('valYZH').textContent = projectionLines.yz_h.toFixed(1);

        // 发送到后端
        fetch('/api/projection_lines', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(projectionLines)
        });
    });
}

// 更新投影线配置（给外部调用）
function updateProjectionLines() {
    document.getElementById('valXYH').textContent = projectionLines.xy_h.toFixed(1);
    document.getElementById('valXYV').textContent = projectionLines.xy_v.toFixed(1);
    document.getElementById('valXZH').textContent = projectionLines.xz_h.toFixed(1);
    document.getElementById('valYZH').textContent = projectionLines.yz_h.toFixed(1);
}

// 处理点云更新
function handlePointCloudUpdate(data) {
    document.getElementById('pointCount').textContent = '点数: ' + data.num_points;
    
    // 绘制三个投影
    drawProjection(xyCtx, data.projections.xy, 'xy', data.lines);
    drawProjection(xzCtx, data.projections.xz, 'xz', data.lines);
    drawProjection(yzCtx, data.projections.yz, 'yz', data.lines);
}

// 绘制投影
function drawProjection(ctx, points, plane, lines) {
    const canvas = ctx.canvas;
    const width = canvas.width;
    const height = canvas.height;
    
    // 清空画布
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);
    
    // 各平面对应的轴标签（横轴, 纵轴）
    const axisLabels = {
        'xy': ['X', 'Y'],
        'xz': ['X', 'Z'],
        'yz': ['Y', 'Z']
    };
    
    if (points.length === 0) {
        // 没有数据时只画坐标轴和标签
        ctx.strokeStyle = '#666';
        ctx.lineWidth = 1;
        const margin = 20;
        const origin = { x: margin, y: height - margin };
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(origin.x + 30, origin.y);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(origin.x, origin.y - 30);
        ctx.stroke();
        ctx.fillStyle = '#fff';
        ctx.font = '12px monospace';
        ctx.fillText(axisLabels[plane][0], origin.x + 35, origin.y + 5);
        ctx.fillText(axisLabels[plane][1], origin.x - 5, origin.y - 35);
        return;
    }
    
    // 计算边界
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    
    points.forEach(p => {
        if (p[0] < minX) minX = p[0];
        if (p[0] > maxX) maxX = p[0];
        if (p[1] < minY) minY = p[1];
        if (p[1] > maxY) maxY = p[1];
    });
    
    // 添加边距
    const rangeX = maxX - minX || 1;
    const rangeY = maxY - minY || 1;
    const margin = 20;
    const scale = Math.min((width - 2 * margin) / rangeX, (height - 2 * margin) / rangeY);
    
    // 转换函数
    const toScreen = (x, y) => {
        return {
            x: margin + (x - minX) * scale,
            y: height - margin - (y - minY) * scale
        };
    };
    
    // 绘制点云
    ctx.fillStyle = '#00ff00';
    points.forEach(p => {
        const screen = toScreen(p[0], p[1]);
        ctx.fillRect(screen.x, screen.y, 2, 2);
    });
    
    // 绘制参考线（实线）
    ctx.strokeStyle = '#ff0000';
    ctx.lineWidth = 2;
    
    if (plane === 'xy') {
        // XY 平面：水平线 + 垂直线
        const y = lines.xy_h;
        const p1 = toScreen(minX, y);
        const p2 = toScreen(maxX, y);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
        
        const x = lines.xy_v;
        const p3 = toScreen(x, minY);
        const p4 = toScreen(x, maxY);
        ctx.beginPath();
        ctx.moveTo(p3.x, p3.y);
        ctx.lineTo(p4.x, p4.y);
        ctx.stroke();
    } else if (plane === 'xz') {
        // XZ 平面：只画水平线
        const z = lines.xz_h;
        const p1 = toScreen(minX, z);
        const p2 = toScreen(maxX, z);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
    } else if (plane === 'yz') {
        // YZ 平面：只画水平线（横轴是 Y，用 minX/maxX）
        const z = lines.yz_h;
        const p1 = toScreen(minX, z);
        const p2 = toScreen(maxX, z);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
    }
    
    // 绘制坐标轴
    ctx.strokeStyle = '#666';
    ctx.lineWidth = 1;
    
    const origin = toScreen(minX, minY);
    // 横轴
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(origin.x + 30, origin.y);
    ctx.stroke();
    // 纵轴
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(origin.x, origin.y - 30);
    ctx.stroke();
    
    // 标签（根据投影平面变化）
    ctx.fillStyle = '#fff';
    ctx.font = '12px monospace';
    ctx.fillText(axisLabels[plane][0], origin.x + 35, origin.y + 5);
    ctx.fillText(axisLabels[plane][1], origin.x - 5, origin.y - 35);
}

// 切换 Tab
function switchTab(tabName) {
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.remove('active');
        if (btn.dataset.tab === tabName) {
            btn.classList.add('active');
        }
    });
    
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('active');
    });
    
    document.getElementById(tabName + '-tab').classList.add('active');
}

// 添加日志
function addLog(message) {
    console.log('[LOG] ' + message);
    // 可以在页面上显示日志，这里简化为 console
}
