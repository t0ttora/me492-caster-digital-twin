const tasks = [...document.querySelectorAll('.task')];
const search = document.querySelector('#task-search');
const filters = [...document.querySelectorAll('[data-filter]')];
let filter = 'all';
function updateTasks() {
  const query = search.value.toLocaleLowerCase('tr').trim();
  let visible = 0;
  for (const task of tasks) {
    const match = (filter === 'all' || task.dataset.status === filter || (filter === 'gate' && task.dataset.kind === 'gate')) && task.textContent.toLocaleLowerCase('tr').includes(query);
    task.hidden = !match;
    if (match) visible++;
  }
  document.querySelector('#task-count').textContent = `${visible} / ${tasks.length} görev gösteriliyor`;
  document.querySelector('#empty-tasks').hidden = visible > 0;
}
if (search) {
  search.addEventListener('input', updateTasks);
  for (const button of filters) button.addEventListener('click', () => {
    filter = button.dataset.filter;
    for (const item of filters) item.setAttribute('aria-pressed', String(item === button));
    updateTasks();
  });
  updateTasks();
}
const navLinks = [...document.querySelectorAll('.header nav a')];
if ('IntersectionObserver' in window) {
  const observer = new IntersectionObserver(entries => {
    for (const entry of entries) if (entry.isIntersecting) {
      for (const link of navLinks) {
        const active = link.hash === `#${entry.target.id}`;
        link.classList.toggle('active', active);
        if (active) link.setAttribute('aria-current', 'location');
        else link.removeAttribute('aria-current');
      }
    }
  }, {rootMargin: '-20% 0px -60% 0px'});
  for (const link of navLinks) { const section = document.querySelector(link.hash); if (section) observer.observe(section); }
}

for (const link of document.querySelectorAll('.roadmap-task')) link.addEventListener('click', () => {
  filter = 'all';
  search.value = '';
  for (const button of filters) button.setAttribute('aria-pressed', String(button.dataset.filter === 'all'));
  updateTasks();
  const task = document.getElementById(link.dataset.task);
  if (task) task.open = true;
});
