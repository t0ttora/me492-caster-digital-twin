/** Pure presentation rules; closing a GitHub issue never proves a research gate. */
export function issueStatus(issue) {
  const labels = (issue.labels || []).map(label => typeof label === 'string' ? label : label.name);
  const statuses = [...new Set(labels.filter(label => /^status:(planned|in-progress|blocked|verified)$/.test(label)).map(label => label.slice(7)))];
  if (statuses.length > 1) return 'conflicting';
  if (issue.state === 'closed') return statuses[0] === 'verified' ? 'verified' : 'closed';
  return statuses[0] === 'verified' ? 'unlabelled' : (statuses[0] || 'unlabelled');
}
export function matchesFilter(text, status, filter, query) {
  return (filter === 'all' || status === filter) && text.toLowerCase().includes(query.trim().toLowerCase());
}
export function validIssue(issue) {
  return Number.isSafeInteger(issue.number) && issue.number > 0 && ['open', 'closed'].includes(issue.state)
    && typeof issue.title === 'string' && Array.isArray(issue.labels)
    && issue.labels.every(label => typeof label === 'string' || (label && typeof label.name === 'string'))
    && issue.html_url === `https://github.com/t0ttora/me492-caster-digital-twin/issues/${issue.number}`;
}
export function formatDate(value) {
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? 'Date unavailable' : new Intl.DateTimeFormat('en-GB', {day:'numeric', month:'short', year:'numeric', timeZone:'UTC'}).format(date);
}
