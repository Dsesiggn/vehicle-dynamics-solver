// Run before the stylesheet so a saved dark preference applies on first paint.
(() => {
  const storageKey = 'suspension-studio.theme.v1';
  let theme = 'light';
  try {
    const saved = localStorage.getItem(storageKey);
    if (saved === 'dark' || saved === 'light') theme = saved;
  } catch { /* Theme switching still works when browser storage is unavailable. */ }

  function applyTheme() {
    document.documentElement.dataset.theme = theme;
    const toggle = document.getElementById('theme-toggle');
    if (toggle) {
      toggle.setAttribute('aria-checked', String(theme === 'dark'));
      toggle.title = theme === 'dark' ? 'Switch to light mode' : 'Switch to dark mode';
    }
    document.querySelector('meta[name="theme-color"]')?.setAttribute('content', theme === 'dark' ? '#171c26' : '#f7f7f8');
  }

  applyTheme();
  document.addEventListener('DOMContentLoaded', () => {
    applyTheme();
    document.getElementById('theme-toggle').addEventListener('click', () => {
      theme = theme === 'dark' ? 'light' : 'dark';
      applyTheme();
      try { localStorage.setItem(storageKey, theme); } catch { /* Keep the in-memory choice. */ }
      // Redraw canvas colors without replacing the design or resetting its pose.
      document.dispatchEvent(new Event('studio-theme-change'));
    });
  }, { once: true });
})();
