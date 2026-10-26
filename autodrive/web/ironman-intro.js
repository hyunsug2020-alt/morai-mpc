(() => {
  const intro = document.getElementById('ironmanIntro');
  const video = document.getElementById('ironmanIntroVideo');
  const play = document.getElementById('ironmanIntroPlay');
  const skip = document.getElementById('ironmanIntroSkip');
  const enter = document.getElementById('ironmanIntroEnter');
  const sound = document.getElementById('ironmanIntroSound');
  const replay = document.getElementById('introReplay');
  if (!intro || !video) return;
  const isActive = document.documentElement.dataset.theme === 'ironman';
  const sessionKey = 'morai-ironman-intro-seen';
  let closed = true;

  function closeIntro() {
    if (closed) return;
    closed = true;
    video.pause();
    intro.classList.add('closing');
    document.body.classList.remove('ironman-intro-open');
    sessionStorage.setItem(sessionKey, '1');
    setTimeout(() => { intro.hidden = true; if (replay) replay.focus(); }, 550);
  }

  function openIntro() {
    closed = false;
    intro.hidden = false;
    intro.classList.remove('closing');
    document.body.classList.add('ironman-intro-open');
    video.currentTime = 0;
    skip.focus();
    video.play().catch(() => { play.hidden = false; });
  }

  skip.onclick = closeIntro;
  enter.onclick = closeIntro;
  if (replay && isActive) { replay.hidden = false; replay.onclick = openIntro; }
  sound.onclick = () => {
    video.muted = !video.muted;
    sound.textContent = video.muted ? '소리 켜기' : '소리 끄기';
  };
  play.onclick = () => video.play().then(() => { play.hidden = true; }).catch(closeIntro);
  video.onended = closeIntro;
  video.onerror = closeIntro;
  video.onplaying = () => { play.hidden = true; };
  document.addEventListener('keydown', (event) => {
    if (intro.hidden || closed) return;
    if (event.key === 'Escape') closeIntro();
    if (event.key === 'Tab') {
      const buttons = [...intro.querySelectorAll('button')].filter(button => !button.hidden);
      if (event.shiftKey && document.activeElement === buttons[0]) {
        event.preventDefault(); buttons.at(-1).focus();
      } else if (!event.shiftKey && document.activeElement === buttons.at(-1)) {
        event.preventDefault(); buttons[0].focus();
      }
    }
  });
  if (isActive && !sessionStorage.getItem(sessionKey) && !matchMedia('(prefers-reduced-motion: reduce)').matches) openIntro();
})();
