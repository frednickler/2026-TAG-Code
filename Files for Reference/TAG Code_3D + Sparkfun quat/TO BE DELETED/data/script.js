// WebSocket connection
const gateway = `ws://${window.location.hostname}/ws`;
let websocket;

// Global variables
let plots;
let airplane1, airplane2;
let map, marker;
let calibration = {
    accelBias: [0, 0, 0],
    gyroBias: [0, 0, 0],
    magBias: [0, 0, 0],
    magScale: [1, 1, 1]
};

window.addEventListener('load', onload);

function onload(event) {
    initWebSocket();
    init3D();
    initMap();
    initUI();
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
    websocket.onerror = function(event) {
        console.error('WebSocket error:', event);
    };
}

function onOpen(event) {
    console.log('Connection opened');
    document.getElementById('stream-toggle').disabled = false;
    document.getElementById('cal-accel-gyro').disabled = false;
    document.getElementById('cal-mag').disabled = false;
    document.getElementById('show-cal-values').disabled = false;
}

function onClose(event) {
    console.log('Connection closed, code:', event.code, 'reason:', event.reason);
    // Disable buttons when connection is lost
    document.getElementById('stream-toggle').disabled = true;
    document.getElementById('cal-accel-gyro').disabled = true;
    document.getElementById('cal-mag').disabled = true;
    document.getElementById('show-cal-values').disabled = true;
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    const data = JSON.parse(event.data);

    console.log("Received WebSocket data:", event.data);

    // Handle calibration values if present
    if (data.calibration) {
        updateCalibrationValues(data.calibration);
    }

    updateSensorTable(data);

    // Update plots
    if (plots) {
        // Add new data point
        const now = Date.now();
        
        // Accelerometer
        if (data.raw_ax !== undefined) {
            plots.accel.x.push(data.raw_ax);
            plots.accel.y.push(data.raw_ay);
            plots.accel.z.push(data.raw_az);
            plots.accel.chart.data.labels.push(now);
            
            // Keep last 100 points
            if (plots.accel.x.length > 100) {
                plots.accel.x.shift();
                plots.accel.y.shift();
                plots.accel.z.shift();
                plots.accel.chart.data.labels.shift();
            }
            
            // Update datasets
            plots.accel.chart.data.datasets[0].data = plots.accel.x;
            plots.accel.chart.data.datasets[1].data = plots.accel.y;
            plots.accel.chart.data.datasets[2].data = plots.accel.z;
            plots.accel.chart.update();
        }

        // Gyroscope
        if (data.raw_gx !== undefined) {
            plots.gyro.x.push(data.raw_gx);
            plots.gyro.y.push(data.raw_gy);
            plots.gyro.z.push(data.raw_gz);
            plots.gyro.chart.data.labels.push(now);
            
            if (plots.gyro.x.length > 100) {
                plots.gyro.x.shift();
                plots.gyro.y.shift();
                plots.gyro.z.shift();
                plots.gyro.chart.data.labels.shift();
            }
            
            plots.gyro.chart.data.datasets[0].data = plots.gyro.x;
            plots.gyro.chart.data.datasets[1].data = plots.gyro.y;
            plots.gyro.chart.data.datasets[2].data = plots.gyro.z;
            plots.gyro.chart.update();
        }

        // Magnetometer
        if (data.raw_mx !== undefined) {
            plots.mag.x.push(data.raw_mx);
            plots.mag.y.push(data.raw_my);
            plots.mag.z.push(data.raw_mz);
            plots.mag.chart.data.labels.push(now);
            
            if (plots.mag.x.length > 100) {
                plots.mag.x.shift();
                plots.mag.y.shift();
                plots.mag.z.shift();
                plots.mag.chart.data.labels.shift();
            }
            
            plots.mag.chart.data.datasets[0].data = plots.mag.x;
            plots.mag.chart.data.datasets[1].data = plots.mag.y;
            plots.mag.chart.data.datasets[2].data = plots.mag.z;
            plots.mag.chart.update();
        }
    }

    // Update quaternion display with color coding
    updateQuaternionDisplay(data);

    // Update 9-DOF "Filtered" Airplane (q9)
    if (airplane1 && data.q9) {
        // Ensure loadedObject is available before applying quaternion
        if (airplane1.quaternion) {
            airplane1.quaternion.set(data.q9[1], data.q9[2], data.q9[3], data.q9[0]); // x, y, z, w
        }
    }

    // Update 6-DOF "Basic" Airplane (q6)
    if (airplane2 && data.q6) {
        // Ensure loadedObject is available before applying quaternion
        if (airplane2.quaternion) {
            airplane2.quaternion.set(data.q6[1], data.q6[2], data.q6[3], data.q6[0]); // x, y, z, w
        }
    }

    // Update Compass Arrow and Digital Heading
    // Always show actual values (default to 0.00 if undefined)
    const magHeading = (typeof data.heading === 'number') ? data.heading : 0.00;
    const trueHeading = (typeof data.true_heading === 'number') ? data.true_heading : 0.00;
    const magnetometerHeading = (typeof data.magnetometer_heading === 'number') ? data.magnetometer_heading : magHeading;
    document.querySelector('.arrow').style.transform = `translateX(-50%) rotate(${magHeading}deg)`;
    document.getElementById('mag-heading').textContent = magHeading.toFixed(2);
    document.getElementById('true-heading').textContent = trueHeading.toFixed(2);
    document.getElementById('magnetometer-heading').textContent = magnetometerHeading.toFixed(2);

    // Update Sensor Data Table
    const fields = {
        'raw-ax': data.raw_ax, 'raw-ay': data.raw_ay, 'raw-az': data.raw_az,
        'raw-gx': data.raw_gx, 'raw-gy': data.raw_gy, 'raw-gz': data.raw_gz,
        'raw-mx': data.raw_mx, 'raw-my': data.raw_my, 'raw-mz': data.raw_mz,
        'cal-ax': data.cal_ax, 'cal-ay': data.cal_ay, 'cal-az': data.cal_az,
        'cal-gx': data.cal_gx, 'cal-gy': data.cal_gy, 'cal-gz': data.cal_gz,
        'cal-mx': data.cal_mx, 'cal-my': data.cal_my, 'cal-mz': data.cal_mz,
        'roll': data.roll, 'pitch': data.pitch, 'yaw': data.yaw,
        'lat': data.lat, 'lon': data.lon, 'alt': data.alt, 'sats': data.sats,
        // Add more detailed sensor data
        'mag-cal': isMagCalibrated() ? 'YES' : 'NO',
        'mag-scale-x': calibration.magScale[0].toFixed(2),
        'mag-scale-y': calibration.magScale[1].toFixed(2),
        'mag-scale-z': calibration.magScale[2].toFixed(2),
        'mag-bias-x': calibration.magBias[0].toFixed(2),
        'mag-bias-y': calibration.magBias[1].toFixed(2),
        'mag-bias-z': calibration.magBias[2].toFixed(2)
    };

    for (const id in fields) {
        const el = document.getElementById(id);
        if (el) {
            let value = fields[id];
            if (typeof value === 'number') {
                if (id.includes('scale') || id.includes('bias')) {
                    el.textContent = value.toFixed(2);
                } else if (id === 'lat' || id === 'lon') {
                    el.textContent = value.toFixed(6);
                } else if (id !== 'sats') {
                    el.textContent = value.toFixed(2);
                } else {
                    el.textContent = value;
                }
            } else {
                el.textContent = value;
            }
        }
    }

    // Update Map Marker
    if (map && marker && data.lat !== undefined && data.lon !== undefined) {
        const newLatLng = [data.lat, data.lon];
        marker.setLatLng(newLatLng);
        map.panTo(newLatLng);
    }
}

function updateSensorTable(data) {
    // Raw Accel
    document.getElementById('raw-ax').textContent = data.raw_ax?.toFixed(2) ?? '0.00';
    document.getElementById('raw-ay').textContent = data.raw_ay?.toFixed(2) ?? '0.00';
    document.getElementById('raw-az').textContent = data.raw_az?.toFixed(2) ?? '0.00';
    // Calibrated Accel
    document.getElementById('cal-ax').textContent = data.cal_ax?.toFixed(2) ?? '0.00';
    document.getElementById('cal-ay').textContent = data.cal_ay?.toFixed(2) ?? '0.00';
    document.getElementById('cal-az').textContent = data.cal_az?.toFixed(2) ?? '0.00';
    // Raw Gyro
    document.getElementById('raw-gx').textContent = data.raw_gx?.toFixed(2) ?? '0.00';
    document.getElementById('raw-gy').textContent = data.raw_gy?.toFixed(2) ?? '0.00';
    document.getElementById('raw-gz').textContent = data.raw_gz?.toFixed(2) ?? '0.00';
    // Calibrated Gyro
    document.getElementById('cal-gx').textContent = data.cal_gx?.toFixed(2) ?? '0.00';
    document.getElementById('cal-gy').textContent = data.cal_gy?.toFixed(2) ?? '0.00';
    document.getElementById('cal-gz').textContent = data.cal_gz?.toFixed(2) ?? '0.00';
    // Raw Mag
    document.getElementById('raw-mx').textContent = data.raw_mx?.toFixed(2) ?? '0.00';
    document.getElementById('raw-my').textContent = data.raw_my?.toFixed(2) ?? '0.00';
    document.getElementById('raw-mz').textContent = data.raw_mz?.toFixed(2) ?? '0.00';
    // Calibrated Mag
    document.getElementById('cal-mx').textContent = data.cal_mx?.toFixed(2) ?? '0.00';
    document.getElementById('cal-my').textContent = data.cal_my?.toFixed(2) ?? '0.00';
    document.getElementById('cal-mz').textContent = data.cal_mz?.toFixed(2) ?? '0.00';
    // Euler
    document.getElementById('roll').textContent = data.roll?.toFixed(2) ?? '0.00';
    document.getElementById('pitch').textContent = data.pitch?.toFixed(2) ?? '0.00';
    document.getElementById('yaw').textContent = data.yaw?.toFixed(2) ?? '0.00';
}

// Function to update calibration values in the UI
function updateCalibrationValues(calibrationData) {
    // Update accelerometer bias values
    if (calibrationData.accelBias) {
        calibration.accelBias = calibrationData.accelBias;
        document.getElementById('accel-bias-x').textContent = calibration.accelBias[0].toFixed(2);
        document.getElementById('accel-bias-y').textContent = calibration.accelBias[1].toFixed(2);
        document.getElementById('accel-bias-z').textContent = calibration.accelBias[2].toFixed(2);
    }
    
    // Update gyroscope bias values
    if (calibrationData.gyroBias) {
        calibration.gyroBias = calibrationData.gyroBias;
        document.getElementById('gyro-bias-x').textContent = calibration.gyroBias[0].toFixed(2);
        document.getElementById('gyro-bias-y').textContent = calibration.gyroBias[1].toFixed(2);
        document.getElementById('gyro-bias-z').textContent = calibration.gyroBias[2].toFixed(2);
    }
    
    // Update magnetometer bias values
    if (calibrationData.magBias) {
        calibration.magBias = calibrationData.magBias;
        document.getElementById('mag-bias-x').textContent = calibration.magBias[0].toFixed(2);
        document.getElementById('mag-bias-y').textContent = calibration.magBias[1].toFixed(2);
        document.getElementById('mag-bias-z').textContent = calibration.magBias[2].toFixed(2);
    }
    
    // Update magnetometer scale values
    if (calibrationData.magScale) {
        calibration.magScale = calibrationData.magScale;
        document.getElementById('mag-scale-x').textContent = calibration.magScale[0].toFixed(2);
        document.getElementById('mag-scale-y').textContent = calibration.magScale[1].toFixed(2);
        document.getElementById('mag-scale-z').textContent = calibration.magScale[2].toFixed(2);
    }
    
    // Show the calibration values container
    document.getElementById('calibration-values-container').style.display = 'block';
}

// Function to check if magnetometer is calibrated
function isMagCalibrated() {
    return calibration.magBias.some(b => Math.abs(b) > 0.01) || 
           calibration.magScale.some(s => Math.abs(s - 1) > 0.01);
}

// Add Chart.js initialization
function initPlots() {
    // Create plot containers
    const plotsContainer = document.createElement('div');
    plotsContainer.className = 'sensor-plots';
    plotsContainer.innerHTML = `
        <div class="plot-container">
            <h3>Accelerometer</h3>
            <canvas id="accelPlot"></canvas>
        </div>
        <div class="plot-container">
            <h3>Gyroscope</h3>
            <canvas id="gyroPlot"></canvas>
        </div>
        <div class="plot-container">
            <h3>Magnetometer</h3>
            <canvas id="magPlot"></canvas>
        </div>
    `;
    const refNode = document.getElementById('cube1-container');
    if (refNode && refNode.parentNode) {
        refNode.parentNode.insertBefore(plotsContainer, refNode);
    } else {
        const leftCol = document.querySelector('.left-column');
        if (leftCol) leftCol.appendChild(plotsContainer);
        else document.body.appendChild(plotsContainer);
    }

    // Initialize charts
    const ctxAccel = document.getElementById('accelPlot').getContext('2d');
    const ctxGyro = document.getElementById('gyroPlot').getContext('2d');
    const ctxMag = document.getElementById('magPlot').getContext('2d');

    // Create datasets
    const datasets = {
        accel: {
            x: [], y: [], z: [],
            chart: new Chart(ctxAccel, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        { label: 'X', borderColor: 'red', data: [] },
                        { label: 'Y', borderColor: 'green', data: [] },
                        { label: 'Z', borderColor: 'blue', data: [] }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            beginAtZero: true,
                            ticks: {
                                stepSize: 1
                            }
                        }
                    }
                }
            })
        },
        gyro: {
            x: [], y: [], z: [],
            chart: new Chart(ctxGyro, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        { label: 'X', borderColor: 'red', data: [] },
                        { label: 'Y', borderColor: 'green', data: [] },
                        { label: 'Z', borderColor: 'blue', data: [] }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            ticks: {
                                stepSize: 10
                            }
                        }
                    }
                }
            })
        },
        mag: {
            x: [], y: [], z: [],
            chart: new Chart(ctxMag, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        { label: 'X', borderColor: 'red', data: [] },
                        { label: 'Y', borderColor: 'green', data: [] },
                        { label: 'Z', borderColor: 'blue', data: [] }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    scales: {
                        y: {
                            ticks: {
                                stepSize: 20
                            }
                        }
                    }
                }
            })
        }
    };

    return datasets;
}

// Add color coding for quaternion values
function updateQuaternionDisplay(data) {
    if (data.q9) {
        const q9 = data.q9;
        const q9Elements = ['q9_w', 'q9_x', 'q9_y', 'q9_z'].map(id => document.getElementById(id));
        q9Elements.forEach((el, i) => {
            const value = q9[i];
            el.textContent = value.toFixed(3);
            // Color coding: red for stuck values, green for normal
            if (Math.abs(value) < 0.001) {
                el.style.color = 'red';
            } else {
                el.style.color = 'green';
            }
        });
    }

    if (data.q6) {
        const q6 = data.q6;
        const q6Elements = ['q6_w', 'q6_x', 'q6_y', 'q6_z'].map(id => document.getElementById(id));
        q6Elements.forEach((el, i) => {
            const value = q6[i];
            el.textContent = value.toFixed(3);
            if (Math.abs(value) < 0.001) {
                el.style.color = 'red';
            } else {
                el.style.color = 'green';
            }
        });
    }
}

// Helper to build a simple procedural airplane using basic geometries
function buildAirplane() {
    const airplane = new THREE.Group();
    const material = new THREE.MeshNormalMaterial();

    // Fuselage (cylinder laid horizontally)
    const fuselageGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.6, 8);
    const fuselage = new THREE.Mesh(fuselageGeo, material);
    fuselage.rotation.z = Math.PI / 2;
    airplane.add(fuselage);

    // Main wings
    const wingGeo = new THREE.BoxGeometry(0.3, 0.02, 0.1);
    const wingL = new THREE.Mesh(wingGeo, material);
    wingL.position.set(0, 0, 0);
    airplane.add(wingL);

    // Tail horizontal stabilizer
    const tailWingGeo = new THREE.BoxGeometry(0.15, 0.015, 0.05);
    const tailWing = new THREE.Mesh(tailWingGeo, material);
    tailWing.position.set(-0.3, 0, 0);
    airplane.add(tailWing);

    // Vertical stabilizer
    const vTailGeo = new THREE.BoxGeometry(0.02, 0.1, 0.05);
    const vTail = new THREE.Mesh(vTailGeo, material);
    vTail.position.set(-0.3, 0.05, 0);
    airplane.add(vTail);

    return airplane;
}

// Three.js 3D Visualization
let scenes = [], renderers = [];

function init3D() {
    const container1 = document.getElementById('cube1-container');
    const container2 = document.getElementById('cube2-container');

    const { scene: scene1, renderer: renderer1, object: obj1 } = create3DScene(container1);
    scenes.push(scene1); renderers.push(renderer1);
    airplane1 = obj1; // Renamed to obj1 as it will be an airplane

    const { scene: scene2, renderer: renderer2, object: obj2 } = create3DScene(container2);
    scenes.push(scene2); renderers.push(renderer2);
    airplane2 = obj2; // Renamed to obj2 as it will be an airplane

    function animate() {
        requestAnimationFrame(animate);
        renderers.forEach((renderer, i) => {
            renderer.render(scenes[i], scenes[i].camera);
        });
    }
    animate();
}

function create3DScene(container) {
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
    camera.position.z = 2; // Adjust camera position for better view of airplane

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // Add ambient light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
    scene.add(ambientLight);

    // Add directional light
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.6);
    directionalLight.position.set(0, 1, 1).normalize();
    scene.add(directionalLight);

    // Create procedural airplane model
    const loadedObject = buildAirplane();
    scene.add(loadedObject);

    // Add a simple grid helper
    const gridHelper = new THREE.GridHelper(2, 10);
    scene.add(gridHelper);

    scene.camera = camera; // Attach camera to scene for easy access in animate loop
    return { scene, renderer, object: loadedObject }; // Return airplane object
}

// Leaflet Map
function initMap() {
    try {
        // Check if Leaflet is available
        if (typeof L === 'undefined') {
            console.warn('Leaflet library not loaded. Map functionality will be disabled.');
            document.getElementById('map-container').innerHTML = '<div class="error-message">Map unavailable - Leaflet library not loaded</div>';
            return;
        }
        
        map = L.map('map').setView([0, 0], 2); // Default view

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        }).addTo(map);

        marker = L.marker([0, 0]).addTo(map)
            .bindPopup('ESP32 Location');
    } catch (error) {
        console.error('Error initializing map:', error);
        document.getElementById('map-container').innerHTML = '<div class="error-message">Map initialization failed</div>';
    }
}

// UI Controls
let streaming = false;
function initUI() {
    document.getElementById('stream-toggle').addEventListener('click', toggleStream);

    document.getElementById('cal-accel-gyro').addEventListener('click', () => {
        websocket.send(JSON.stringify({ command: 'calibrateAccel' }));
        console.log('Sent command: calibrateAccel');
    });

    document.getElementById('cal-mag').addEventListener('click', () => {
        websocket.send(JSON.stringify({ command: 'calibrateMag' }));
        console.log('Sent command: calibrateMag');
    });
    
    document.getElementById('show-cal-values').addEventListener('click', () => {
        websocket.send(JSON.stringify({ command: 'getCalibrationValues' }));
        console.log('Sent command: getCalibrationValues');
    });

    // Initialize plots
    plots = initPlots();
}

function toggleStream() {
    if (!websocket || websocket.readyState !== WebSocket.OPEN) {
        console.log('WebSocket not open, cannot toggle stream');
        return;
    }
    if (!streaming) {
        websocket.send(JSON.stringify({command: 'startStream'}));
        streaming = true;
        document.getElementById('stream-toggle').textContent = 'Stop Stream';
        console.log('Sent startStream command');
    } else {
        websocket.send(JSON.stringify({command: 'stopStream'}));
        streaming = false;
        document.getElementById('stream-toggle').textContent = 'Start Stream';
        console.log('Sent stopStream command');
    }
}
