const canvas = document.getElementById('scene');
const ctx = canvas.getContext('2d');
const statusList = document.getElementById('statusList');
const inventoryList = document.getElementById('inventoryList');
const assignmentList = document.getElementById('assignmentList');
const taskList = document.getElementById('taskList');
const resetButton = document.getElementById('resetButton');
const taskForm = document.getElementById('taskForm');
const taskMessage = document.getElementById('taskMessage');
const taskX = document.getElementById('taskX');
const taskY = document.getElementById('taskY');

const world = { width: 10, height: 10 };

function worldToCanvas(x, y) {
  return {
    x: 50 + (x / world.width) * (canvas.width - 100),
    y: 30 + (y / world.height) * (canvas.height - 60),
  };
}

function canvasToWorld(px, py) {
  const x = ((px - 50) / (canvas.width - 100)) * world.width;
  const y = ((py - 30) / (canvas.height - 60)) * world.height;
  return { x: Math.max(0, Math.min(world.width, x)), y: Math.max(0, Math.min(world.height, y)) };
}

function drawWarehouse(charger) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = 'rgba(144, 181, 255, 0.10)';
  ctx.lineWidth = 1;
  for (let x = 0; x <= 10; x += 1) {
    const px = 50 + (x / world.width) * (canvas.width - 100);
    ctx.beginPath();
    ctx.moveTo(px, 30);
    ctx.lineTo(px, canvas.height - 30);
    ctx.stroke();
  }
  for (let y = 0; y <= 10; y += 1) {
    const py = 30 + (y / world.height) * (canvas.height - 60);
    ctx.beginPath();
    ctx.moveTo(50, py);
    ctx.lineTo(canvas.width - 50, py);
    ctx.stroke();
  }

  const chargerPos = worldToCanvas(charger.x, charger.y);
  ctx.fillStyle = '#4ad6ff';
  ctx.fillRect(chargerPos.x - 18, chargerPos.y - 18, 36, 36);
  ctx.fillStyle = '#03141f';
  ctx.font = 'bold 10px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('CHARGER', chargerPos.x, chargerPos.y - 24);

  const racks = [
    { x: 2.2, y: 8.2, w: 1.3, h: 1.3 },
    { x: 7.3, y: 5.7, w: 1.4, h: 1.2 },
    { x: 3.7, y: 3.4, w: 1.4, h: 1.5 },
  ];

  racks.forEach((rack) => {
    const pos = worldToCanvas(rack.x, rack.y);
    const w = (rack.w / world.width) * (canvas.width - 100);
    const h = (rack.h / world.height) * (canvas.height - 60);
    ctx.fillStyle = 'rgba(122, 156, 196, 0.25)';
    ctx.fillRect(pos.x, pos.y, w, h);
  });
}

function drawTask(task, name) {
  const pos = worldToCanvas(task.x, task.y);
  const radius = task.done ? 10 : 12;
  ctx.beginPath();
  ctx.fillStyle = task.done ? '#3ddc97' : '#f7b955';
  ctx.arc(pos.x, pos.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#081421';
  ctx.font = 'bold 12px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(name, pos.x, pos.y + 28);
}

function drawRobot(robot, name) {
  const pos = worldToCanvas(robot.x, robot.y);

  ctx.beginPath();
  ctx.fillStyle = '#58a6ff';
  ctx.fillRect(pos.x - 12, pos.y - 12, 24, 24);

  ctx.beginPath();
  ctx.fillStyle = '#d9efff';
  ctx.arc(pos.x, pos.y, 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#ebf5ff';
  ctx.font = 'bold 12px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(name, pos.x, pos.y - 20);

  ctx.fillStyle = '#9be7ff';
  ctx.fillRect(pos.x - 14, pos.y + 18, 28, 5);
  ctx.fillStyle = '#4ade80';
  ctx.fillRect(pos.x - 14, pos.y + 18, (robot.battery / 100) * 28, 5);
}

function render(data) {
  drawWarehouse(data.charger || { x: 1.5, y: 9.0 });

  Object.entries(data.tasks || {}).forEach(([name, task]) => {
    drawTask(task, name);
  });

  Object.entries(data.robots || {}).forEach(([name, robot]) => {
    drawRobot(robot, name);
  });

  renderStatus(data);
}

function renderStatus(data) {
  const robotHTML = Object.entries(data.robots || {}).map(([name, robot]) => {
    const task = robot.task ? `Task ${robot.task}` : 'Idle';
    return `
      <div class="status-row">
        <span>${name}</span>
        <span class="value-pill">${task}</span>
      </div>
      <div class="status-row">
        <span>Battery</span>
        <span>${robot.battery}%</span>
      </div>
    `;
  }).join('');

  statusList.innerHTML = robotHTML;

  const inventoryHTML = Object.entries(data.inventory || {}).map(([name, item]) => `
    <div class="inventory-row">
      <span>${name}</span>
      <span class="value-pill">${item.status}</span>
    </div>
    <div class="inventory-row">
      <span>Stock</span>
      <span>${item.stock}</span>
    </div>
  `).join('');
  inventoryList.innerHTML = inventoryHTML;

  const assignmentHTML = Object.entries(data.tasks || {}).map(([taskName, task]) => {
    const assignedRobot = Object.entries(data.robots || {}).find(([, robot]) => robot.task === taskName)?.[0] || 'waiting';
    return `
      <div class="assignment-row">
        <span>${taskName}</span>
        <span class="value-pill">${task.done ? 'done' : assignedRobot}</span>
      </div>
    `;
  }).join('');
  assignmentList.innerHTML = assignmentHTML;

  taskList.replaceChildren();
  Object.entries(data.tasks || {}).forEach(([taskName, task]) => {
    const row = document.createElement('div');
    row.className = 'task-row';

    const heading = document.createElement('div');
    heading.className = 'status-row';
    const taskLabel = document.createElement('strong');
    taskLabel.textContent = taskName;
    const taskState = document.createElement('span');
    taskState.className = 'value-pill';
    taskState.textContent = task.done ? 'done' : 'open';
    heading.append(taskLabel, taskState);

    const description = document.createElement('div');
    description.className = 'task-description';
    description.textContent = `${task.description} · ${task.payload} kg · urgency ${task.urgency}`;
    row.append(heading, description);
    taskList.append(row);
  });
}

async function fetchState() {
  const response = await fetch('/api/state');
  if (!response.ok) throw new Error('Failed to load simulation state');
  return response.json();
}

async function resetSimulation() {
  const response = await fetch('/api/reset', { method: 'POST' });
  if (!response.ok) throw new Error('Failed to reset simulation');
  return response.json();
}

async function createTask(task) {
  const response = await fetch('/api/task', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(task),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || 'Failed to create task');
  return result;
}

canvas.addEventListener('click', async (event) => {
  const rect = canvas.getBoundingClientRect();
  const px = event.clientX - rect.left;
  const py = event.clientY - rect.top;
  const worldPoint = canvasToWorld(px, py);
  taskX.value = worldPoint.x.toFixed(1);
  taskY.value = worldPoint.y.toFixed(1);
  taskMessage.textContent = `Selected location (${taskX.value}, ${taskY.value}) m. Complete the task details and add it.`;
});

taskForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const task = {
    description: document.getElementById('taskDescription').value.trim(),
    x: Number(taskX.value),
    y: Number(taskY.value),
    payload: Number(document.getElementById('taskPayload').value),
    urgency: Number(document.getElementById('taskUrgency').value),
  };
  try {
    const result = await createTask(task);
    taskMessage.textContent = `${result.task} added. Robots will choose a feasible assignment.`;
    render(await fetchState());
  } catch (error) {
    taskMessage.textContent = error.message;
  }
});

async function startLoop() {
  try {
    const data = await fetchState();
    render(data);
  } catch (error) {
    console.error(error);
  }

  setInterval(async () => {
    try {
      const data = await fetchState();
      render(data);
    } catch (error) {
      console.error(error);
    }
  }, 800);
}

resetButton.addEventListener('click', async () => {
  await resetSimulation();
  taskMessage.textContent = '';
  const data = await fetchState();
  render(data);
});

startLoop();
