import {getDocument, GlobalWorkerOptions} from './pdfjs/pdf.min.mjs';
GlobalWorkerOptions.workerSrc = new URL('./pdfjs/pdf.worker.min.mjs', import.meta.url).href;
const stage = document.querySelector('.pdf-stage');
const canvas = document.querySelector('#pdf-canvas');
const status = document.querySelector('#pdf-status');
const pageInput = document.querySelector('#pdf-page');
const prev = document.querySelector('#pdf-prev'), next = document.querySelector('#pdf-next');
const zoomOut = document.querySelector('#pdf-out'), zoomIn = document.querySelector('#pdf-in');
const controls = [pageInput, prev, next, zoomOut, zoomIn];
let pdf, number = 1, zoom = 1, busy = false, pending = false;
async function render() {
  if (!pdf) return;
  if (busy) {pending = true; return;}
  busy = true;
  controls.forEach(control => {control.disabled = true;});
  stage.setAttribute('aria-busy', 'true');
  status.textContent = `Loading page ${number}…`;
  try {
    const page = await pdf.getPage(number);
    const base = page.getViewport({scale:1});
    const scale = (stage.clientWidth - 32) / base.width * zoom;
    const viewport = page.getViewport({scale});
    const ratio = Math.min(devicePixelRatio || 1, 2);
    canvas.width = Math.ceil(viewport.width * ratio);
    canvas.height = Math.ceil(viewport.height * ratio);
    canvas.style.width = `${viewport.width}px`;
    canvas.style.height = `${viewport.height}px`;
    await page.render({canvas, canvasContext:canvas.getContext('2d'), viewport, transform:[ratio,0,0,ratio,0,0]}).promise;
    const text = await page.getTextContent();
    document.querySelector('#pdf-text').textContent = text.items.map(item => item.str || '').join(' ');
    canvas.setAttribute('aria-label', `Page ${number} of ${pdf.numPages}. Use Read page as text for the text version.`);
    pageInput.value = number;
    document.querySelector('#pdf-total').textContent = `of ${pdf.numPages}`;
    document.querySelector('#pdf-zoom').textContent = zoom === 1 ? 'Fit width' : `${Math.round(zoom * 100)}%`;
    status.textContent = `Page ${number} of ${pdf.numPages}`;
  } catch {
    status.textContent = 'This page could not be displayed. Try again or open the original PDF below.';
  } finally {
    busy = false;
    stage.setAttribute('aria-busy', 'false');
    controls.forEach(control => {control.disabled = false;});
    prev.disabled = number <= 1; next.disabled = number >= pdf.numPages;
    zoomOut.disabled = zoom <= 0.75; zoomIn.disabled = zoom >= 2;
    if (pending) {pending = false; render();}
  }
}
prev.addEventListener('click', () => {number--; stage.scrollTop = 0; render();});
next.addEventListener('click', () => {number++; stage.scrollTop = 0; render();});
pageInput.addEventListener('change', () => {
  const value = Number(pageInput.value);
  if (Number.isInteger(value) && value >= 1 && value <= pdf.numPages) {number = value; stage.scrollTop = 0; render();}
  else pageInput.value = number;
});
zoomOut.addEventListener('click', () => {zoom = Math.max(0.75, zoom - 0.25); render();});
zoomIn.addEventListener('click', () => {zoom = Math.min(2, zoom + 0.25); render();});
let resize;
window.addEventListener('resize', () => {clearTimeout(resize); resize = setTimeout(render, 150);});
try {
  pdf = await getDocument({url:stage.dataset.pdf, isEvalSupported:false,
    standardFontDataUrl:new URL('./pdfjs/standard_fonts/', import.meta.url).href,
    wasmUrl:new URL('./pdfjs/wasm/', import.meta.url).href}).promise;
  pageInput.max = pdf.numPages;
  await render();
} catch {
  status.textContent = 'The document could not be loaded. Open the original PDF or download it above.';
}
