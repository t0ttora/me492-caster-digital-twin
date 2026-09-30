import { issueStatus, matchesFilter, validIssue, formatDate } from './status.mjs';

const API = 'https://api.github.com/repos/t0ttora/me492-caster-digital-twin';
const REPO = 'https://github.com/t0ttora/me492-caster-digital-twin';
const labels = {'in-progress':'In progress',planned:'Planned',blocked:'Blocked',verified:'Verified',closed:'Closed · review',unlabelled:'Unlabelled',conflicting:'Check labels'};
const rows = [...document.querySelectorAll('.week-row')];
const search = document.querySelector('.search input');
const filters = [...document.querySelectorAll('[data-filter]')];
const showAll = document.querySelector('#show-all');
const resultCount = document.querySelector('#result-count');
const refresh = document.querySelector('#refresh');
const syncStatus = document.querySelector('#sync-status');
let filter = 'all';
let expanded = false;
let lastIssueSync = null;

function applyFilters() {
  const matches = rows.filter(row => matchesFilter(row.querySelector('.week-title').textContent, row.dataset.status, filter, search.value));
  const limited = !expanded && filter === 'all' && !search.value.trim();
  rows.forEach(row => { row.hidden = !matches.includes(row) || (limited && matches.indexOf(row) >= 6); });
  document.querySelector('#no-results').hidden = matches.length !== 0;
  showAll.hidden = filter !== 'all' || !!search.value.trim() || matches.length <= 6;
  showAll.textContent = expanded ? 'Show first 6 weeks ↑' : `Show all ${matches.length} weeks ↓`;
  resultCount.textContent = `${limited ? Math.min(6, matches.length) : matches.length} of ${rows.length} work packages shown`;
}
filters.forEach(button => button.addEventListener('click', () => {
  filter = button.dataset.filter;
  filters.forEach(item => item.setAttribute('aria-pressed', String(item === button)));
  applyFilters();
}));
search.addEventListener('input', applyFilters);
showAll.addEventListener('click', () => { expanded = !expanded; applyFilters(); });
document.querySelector('.filter-tools').hidden = false;
refresh.hidden = false;
applyFilters();

async function getJson(path, signal) {
  const response = await fetch(`${API}${path}`, {headers:{Accept:'application/vnd.github+json'}, signal});
  if (!response.ok) throw new Error(`GitHub HTTP ${response.status}`);
  const data = await response.json();
  if (!Array.isArray(data)) throw new Error('Unexpected GitHub response');
  return data;
}
async function loadIssues(signal) {
  const all = [];
  for (let page = 1; page <= 10; page++) {
    const batch = await getJson(`/issues?state=all&per_page=100&page=${page}`, signal);
    all.push(...batch.filter(issue => !issue.pull_request));
    if (batch.length < 100) {
      if (!all.every(validIssue)) throw new Error('Unexpected issue data');
      return all;
    }
  }
  throw new Error('Issue list needs more pages than the dashboard supports');
}
function setStatus(element, value) {
  element.className = `status ${value}`;
  element.replaceChildren();
  const dot = document.createElement('i'); dot.setAttribute('aria-hidden', 'true');
  element.append(dot, document.createTextNode(labels[value] || 'Unlabelled'));
}
function updateIssues(issues) {
  const byNumber = new Map(issues.map(issue => [issue.number, issue]));
  for (const entry of document.querySelectorAll('[data-issue]')) {
    const issue = byNumber.get(Number(entry.dataset.issue));
    const value = issue ? issueStatus(issue) : 'unlabelled';
    setStatus(entry.querySelector('.status'), value);
    if (entry.matches('.week-row')) entry.dataset.status = value;
  }
  lastIssueSync = new Date();
  const time = new Intl.DateTimeFormat('en-GB', {hour:'2-digit',minute:'2-digit',timeZone:'UTC'}).format(lastIssueSync);
  syncStatus.textContent = `Live issue labels · checked ${formatDate(lastIssueSync)} at ${time} UTC. Research content remains the dated source record.`;
  document.querySelector('.sync-bar').classList.add('live');
  applyFilters();
}
function updateCommits(commits) {
  const list = document.querySelector('#commit-list');
  const fragment = document.createDocumentFragment();
  for (const commit of commits) {
    if (!/^[a-f0-9]{40}$/.test(commit.sha) || commit.html_url !== `${REPO}/commit/${commit.sha}` || typeof commit.commit?.message !== 'string') throw new Error('Unexpected commit data');
    const li = document.createElement('li');
    const link = document.createElement('a'); link.href = commit.html_url; link.textContent = commit.commit.message.split('\n')[0];
    const meta = document.createElement('span'); meta.className = 'mono'; meta.textContent = `${formatDate(commit.commit.committer?.date)} · ${commit.sha.slice(0,7)}`;
    li.append(link, meta); fragment.append(li);
  }
  if (!commits.length) throw new Error('Empty repository history');
  list.replaceChildren(fragment);
  document.querySelector('#commit-state').textContent = `Latest main commits from GitHub · checked ${formatDate(new Date())}`;
}
async function refreshGithub() {
  refresh.disabled = true;
  refresh.textContent = 'Checking GitHub…';
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 12000);
  const [issues, commits] = await Promise.allSettled([loadIssues(controller.signal), getJson('/commits?sha=main&per_page=5', controller.signal)]);
  clearTimeout(timeout);
  try {
    if (issues.status === 'rejected') throw issues.reason;
    updateIssues(issues.value);
  } catch {
    document.querySelector('.sync-bar').classList.remove('live');
    syncStatus.textContent = lastIssueSync
      ? `GitHub refresh unavailable. Keeping issue labels last checked ${formatDate(lastIssueSync)}; open GitHub for current status.`
      : 'GitHub refresh unavailable. Showing snapshot issue labels checked 30 Sep 2026; open GitHub for current status.';
  }
  try {
    if (commits.status === 'rejected') throw commits.reason;
    updateCommits(commits.value);
  } catch {
    document.querySelector('#commit-state').textContent = 'GitHub refresh unavailable · keeping the last displayed repository history';
  }
  refresh.disabled = false;
  refresh.innerHTML = 'Refresh GitHub <span aria-hidden="true">↻</span>';
}
refresh.addEventListener('click', refreshGithub);
refreshGithub();
