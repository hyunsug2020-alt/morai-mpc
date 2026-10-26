// Click feedback for the spider (black/red, Iron-Man HUD flavoured) theme:
// squash + glow on the button, a red ripple, and a small spider-web burst at
// the pointer. Disabled for the default theme and for reduced motion.
(() => {
  const theme = document.documentElement.dataset.theme || 'default';
  if (theme === 'default' || matchMedia('(prefers-reduced-motion: reduce)').matches) return;

  const style = document.createElement('style');
  style.textContent = `
    .fx-hit { animation: fxHit .32s cubic-bezier(.2,.9,.3,1.4); }
    @keyframes fxHit {
      0% { transform: scale(1); filter: none; }
      35% { transform: scale(.94) skewX(-2deg); filter: drop-shadow(0 0 10px #ff2a3d) brightness(1.35); }
      100% { transform: scale(1); filter: none; }
    }
    .fx-ripple { position: fixed; z-index: 3000; pointer-events: none; width: 14px; height: 14px; margin: -7px 0 0 -7px;
      border: 2px solid #ff2a3d; border-radius: 50%; box-shadow: 0 0 14px #ff2a3d, inset 0 0 8px #ff2a3d;
      animation: fxRipple .55s ease-out forwards; }
    @keyframes fxRipple { to { transform: scale(9); opacity: 0; border-width: 1px; } }
    .fx-web { position: fixed; z-index: 3001; pointer-events: none; width: 120px; height: 120px; margin: -60px 0 0 -60px;
      animation: fxWeb .6s ease-out forwards; }
    @keyframes fxWeb { 0% { transform: scale(.2) rotate(-20deg); opacity: 1; } 100% { transform: scale(1.15) rotate(8deg); opacity: 0; } }
    .fx-flash { position: fixed; inset: 0; z-index: 2999; pointer-events: none;
      background: radial-gradient(circle at var(--x) var(--y), #ff2a3d33, transparent 45%); animation: fxFlash .35s ease-out forwards; }
    @keyframes fxFlash { to { opacity: 0; } }
  `;
  document.head.appendChild(style);

  const web = () => {
    let lines = '';
    for (let i = 0; i < 8; i++) {
      const a = i * Math.PI / 4;
      lines += `<line x1="60" y1="60" x2="${60 + Math.cos(a) * 56}" y2="${60 + Math.sin(a) * 56}"/>`;
    }
    let rings = '';
    for (const r of [14, 28, 42, 54]) {
      let d = '';
      for (let i = 0; i <= 8; i++) {
        const a = i * Math.PI / 4, x = 60 + Math.cos(a) * r, y = 60 + Math.sin(a) * r;
        d += (i ? ' L' : 'M') + x.toFixed(1) + ' ' + y.toFixed(1);
      }
      rings += `<path d="${d}"/>`;
    }
    return `<svg viewBox="0 0 120 120" fill="none" stroke="#ff4055" stroke-width="1.2" opacity=".95">${lines}${rings}</svg>`;
  };

  document.addEventListener('pointerdown', event => {
    const target = event.target.closest('button, .btn, .hero-actions a, a.tag, [data-action]');
    if (!target || target.disabled) return;
    target.classList.remove('fx-hit'); void target.offsetWidth; target.classList.add('fx-hit');
    const ripple = document.createElement('span');
    ripple.className = 'fx-ripple'; ripple.style.left = event.clientX + 'px'; ripple.style.top = event.clientY + 'px';
    const burst = document.createElement('div');
    burst.className = 'fx-web'; burst.style.left = event.clientX + 'px'; burst.style.top = event.clientY + 'px';
    burst.innerHTML = web();
    const flash = document.createElement('div');
    flash.className = 'fx-flash'; flash.style.setProperty('--x', event.clientX + 'px'); flash.style.setProperty('--y', event.clientY + 'px');
    document.body.append(ripple, burst, flash);
    setTimeout(() => { ripple.remove(); burst.remove(); flash.remove(); }, 650);
  }, { passive: true });
})();
