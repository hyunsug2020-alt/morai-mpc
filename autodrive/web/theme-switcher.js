(() => {
  const THEMES = [
    { id: 'default', label: '기본' },
    { id: 'spider', label: '스파이더맨' },
    { id: 'ironman', label: '아이언맨' },
  ];
  const current = document.documentElement.dataset.theme || 'default';
  const EYEBROW = {
    default: 'MORAI · RGB TEACHER–STUDENT',
    spider: 'SPIDER / MORAI · RGB TEACHER–STUDENT',
    ironman: 'IRON / MORAI · RGB TEACHER–STUDENT',
  };
  const eyebrow = document.getElementById('themeEyebrow');
  if (eyebrow && EYEBROW[current]) eyebrow.textContent = EYEBROW[current];

  const style = document.createElement('style');
  style.textContent = `
    #themeSwitcher { position:fixed; top:16px; right:16px; z-index:2000; font-family:Pretendard,"Noto Sans KR",system-ui,sans-serif; }
    #themeSwitcher button { cursor:pointer; }
    #themeSwitcherToggle { display:flex; align-items:center; gap:7px; padding:9px 13px; border:1px solid rgba(255,255,255,.18); border-radius:999px; background:rgba(10,10,14,.72); backdrop-filter:blur(6px); color:#f2f1ef; font-size:12px; font-weight:700; box-shadow:0 8px 24px rgba(0,0,0,.28); }
    #themeSwitcherToggle:hover { background:rgba(10,10,14,.88); }
    #themeSwitcherMenu { position:absolute; top:calc(100% + 8px); right:0; min-width:150px; padding:6px; border:1px solid rgba(255,255,255,.14); border-radius:14px; background:rgba(12,12,16,.92); backdrop-filter:blur(8px); box-shadow:0 14px 34px rgba(0,0,0,.35); }
    #themeSwitcherMenu[hidden] { display:none; }
    #themeSwitcherMenu button { display:flex; align-items:center; justify-content:space-between; gap:10px; width:100%; padding:9px 11px; border:0; border-radius:9px; background:transparent; color:#eceae6; font-size:12.5px; font-weight:600; text-align:left; }
    #themeSwitcherMenu button:hover { background:rgba(255,255,255,.08); }
    #themeSwitcherMenu button[aria-current="true"] { color:#ffcf6b; }
    #themeSwitcherMenu button span.mark { opacity:0; }
    #themeSwitcherMenu button[aria-current="true"] span.mark { opacity:1; }
    @media (max-width:520px) { #themeSwitcher { top:10px; right:10px; } #themeSwitcherToggle { padding:8px 11px; font-size:11px; } }
    body.spider-intro-open #themeSwitcher, body.ironman-intro-open #themeSwitcher { display:none; }
  `;
  document.head.appendChild(style);

  const wrap = document.createElement('div');
  wrap.id = 'themeSwitcher';
  const currentLabel = (THEMES.find(t => t.id === current) || THEMES[0]).label;
  wrap.innerHTML = `
    <button id="themeSwitcherToggle" type="button" aria-haspopup="true" aria-expanded="false">🎨 ${currentLabel} 테마</button>
    <div id="themeSwitcherMenu" hidden role="menu"></div>
  `;
  document.body.appendChild(wrap);

  const toggle = document.getElementById('themeSwitcherToggle');
  const menu = document.getElementById('themeSwitcherMenu');
  menu.innerHTML = THEMES.map(t => `
    <button type="button" role="menuitemradio" aria-current="${t.id === current}" data-theme-id="${t.id}">
      ${t.label}<span class="mark">✓</span>
    </button>
  `).join('');

  function closeMenu() { menu.hidden = true; toggle.setAttribute('aria-expanded', 'false'); }
  function openMenu() { menu.hidden = false; toggle.setAttribute('aria-expanded', 'true'); }

  toggle.onclick = () => { menu.hidden ? openMenu() : closeMenu(); };
  document.addEventListener('click', (event) => {
    if (!wrap.contains(event.target)) closeMenu();
  });
  document.addEventListener('keydown', (event) => { if (event.key === 'Escape') closeMenu(); });

  menu.querySelectorAll('button[data-theme-id]').forEach((button) => {
    button.onclick = () => {
      const id = button.dataset.themeId;
      if (id === current) { closeMenu(); return; }
      localStorage.setItem('morai-theme', id);
      location.reload();
    };
  });
})();
