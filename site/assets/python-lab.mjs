const buttons = [...document.querySelectorAll('[data-python-example]')];
const status = document.querySelector('#python-status');
const output = document.querySelector('#python-output');
const cancel = document.querySelector('#cancel-run');
const palette = ['#9ed6ff', '#edb882'];
let worker;
let csv;
let example;

function setBusy(busy) {
  for (const button of buttons) {
    button.disabled = busy;
    button.textContent = busy && button.dataset.pythonExample === example ? 'Running Python…' : button.dataset.label;
  }
  cancel.hidden = !busy;
}

function drawPlot(canvas, lines, equalScale = false) {
  const context = canvas.getContext('2d');
  const [width, height] = [canvas.width, canvas.height];
  const [left, right, top, bottom] = [65, width - 33, 18, height - 35];
  const pairs = lines.flatMap(line => line.points);
  const xs = pairs.map(point => point[0]);
  const ys = pairs.map(point => point[1]);
  const xmid = (Math.min(...xs) + Math.max(...xs)) / 2;
  const ymid = (Math.min(...ys) + Math.max(...ys)) / 2;
  const xspan = Math.max(Math.max(...xs) - Math.min(...xs), .001);
  const yspan = Math.max(Math.max(...ys) - Math.min(...ys), .001);
  const ratio = (right - left) / (bottom - top);
  const dx = 1.06 * (equalScale ? Math.max(xspan, yspan * ratio) : xspan);
  const dy = 1.06 * (equalScale ? Math.max(yspan, xspan / ratio) : yspan);
  const xmin = xmid - dx / 2;
  const ymin = ymid - dy / 2;
  context.fillStyle = '#171918';
  context.fillRect(0, 0, width, height);
  context.font = '15px monospace';
  context.fillStyle = '#c0c5bc';
  context.strokeStyle = '#414941';
  context.lineWidth = 1;
  context.beginPath();
  for (let division = 0; division <= 4; division++) {
    const fraction = division / 4;
    const x = left + (right - left) * fraction;
    const y = bottom - (bottom - top) * fraction;
    context.moveTo(x, top);
    context.lineTo(x, bottom);
    context.moveTo(left, y);
    context.lineTo(right, y);
    context.textAlign = 'center';
    context.fillText((xmin + dx * fraction).toFixed(2), x, height - 10);
    context.textAlign = 'right';
    context.fillText((ymin + dy * fraction).toFixed(2), left - 5, y + 5);
  }
  context.stroke();
  for (const line of lines) {
    context.beginPath();
    line.points.forEach(([x, y], index) => {
      const px = left + (x - xmin) / dx * (right - left);
      const py = bottom - (y - ymin) / dy * (bottom - top);
      if (index === 0) context.moveTo(px, py);
      else context.lineTo(px, py);
    });
    context.lineWidth = line.dashed ? 1.5 : 2.75;
    context.setLineDash(line.dashed ? [5, 5] : []);
    context.strokeStyle = line.color;
    context.stroke();
  }
  context.setLineDash([]);
}

function showRun(result) {
  const series = result.series;
  const comparesProfiles = series.length > 1;
  const metrics = document.querySelector('#python-metrics');
  metrics.replaceChildren(...series.map((run, index) => {
    const row = document.createElement('tr');
    const name = document.createElement('th');
    name.scope = 'row';
    name.textContent = run.name;
    name.style.color = palette[index];
    row.append(name);
    for (const value of [
      String(run.samples.length), run.duration.toFixed(2),
      run.pathRmse == null ? '—' : (run.pathRmse * 1000).toFixed(2),
      run.completion == null ? '—' : run.completion.toFixed(2), run.peakTorque.toFixed(3),
    ]) {
      const cell = document.createElement('td');
      cell.textContent = value;
      row.append(cell);
    }
    return row;
  }));
  const path = series.map((run, index) => ({color: palette[index], points: run.samples.map(row => row.slice(1, 3))}));
  if (comparesProfiles) {
    path.unshift({color: '#a7ada7', dashed: true, points: series[0].samples.map(row => row.slice(4, 6))});
  }
  document.querySelector('#reference-legend').hidden = !comparesProfiles;
  drawPlot(document.querySelector('#plot-path'), path, true);
  drawPlot(document.querySelector('#plot-caster'), series.map((run, i) => ({
    color: palette[i], points: run.samples.map(([t, , , caster]) => [t, caster]),
  })));
  csv = result.csv;
  const heading = document.querySelector('#run-title');
  heading.textContent = comparesProfiles ? 'S-turn profile comparison · generated results' : 'Equal-torque baseline · generated results';
  output.hidden = false;
  status.textContent = 'Python run complete.';
  heading.focus({preventScroll: true});
  output.scrollIntoView({block: 'start'});
}

for (const button of buttons) {
  button.dataset.label = button.textContent;
  button.addEventListener('click', () => {
    example = button.dataset.pythonExample;
    setBusy(true);
    output.hidden = true;
    status.textContent = worker ? 'Running the Python example…' : 'Loading Python and NumPy…';
    if (!worker) {
      try {
        worker = new Worker(new URL('./python-worker.mjs', import.meta.url), {type: 'module'});
      } catch {
        status.textContent = 'This browser could not start Python. The recorded results are linked above.';
        setBusy(false);
        return;
      }
      worker.addEventListener('message', ({data}) => {
        if (data.status) status.textContent = data.status;
        else {
          if (data.result) showRun(data.result);
          else status.textContent = 'Python could not run. ' + data.error.split('\n').filter(Boolean).at(-1);
          setBusy(false);
        }
      });
      worker.addEventListener('error', () => {
        worker.terminate();
        worker = null;
        status.textContent = 'The Python runtime could not load. Retry or open the recorded results.';
        setBusy(false);
      });
    }
    worker.postMessage(example);
  });
}
setBusy(false);
cancel.addEventListener('click', () => {
  worker?.terminate();
  worker = null;
  status.textContent = 'Python run stopped.';
  setBusy(false);
});
document.querySelector('#download-run').addEventListener('click', () => {
  const url = URL.createObjectURL(new Blob([csv], {type: 'text/csv'}));
  const link = document.createElement('a');
  link.href = url;
  link.download = 'arche-' + example + '.csv';
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
