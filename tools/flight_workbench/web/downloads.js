/* Shared feedback for board log downloads; kept outside periodically redrawn lists. */
(function () {
  'use strict';
  const active = new Map();
  let panel;

  function node(tag, className, text) {
    const item = document.createElement(tag);
    if (className) item.className = className;
    if (text != null) item.textContent = text;
    return item;
  }
  function size(bytes) {
    return (bytes / 1048576).toFixed(2) + ' MiB';
  }
  function updateButtons(url) {
    document.querySelectorAll('[data-log-download-url]').forEach(button => {
      if (button.dataset.logDownloadUrl !== url) return;
      button.disabled = active.has(url);
      button.textContent = button.disabled ? '下载中…' : button.dataset.logDownloadLabel;
    });
  }
  function start(url, filename, expectedSize, displayName) {
    const snapshot = filename.toLowerCase().endsWith('.bag.active');
    if (snapshot) expectedSize = undefined; // Recording size may change after listing.
    if (active.has(url)) return;
    if (!panel) {
      panel = node('section', 'log-downloads');
      panel.setAttribute('aria-label', '日志下载状态');
      panel.append(node('h2', '', '日志下载'));
      document.body.append(panel);
    }
    const card = node('div', 'log-download-card');
    const title = node('div', 'log-download-name', displayName || filename);
    const status = node('p', 'log-download-status', '正在从板端读取日志…');
    status.setAttribute('role', 'status');
    const progress = node('progress');
    progress.max = 100;
    progress.setAttribute('aria-label', '浏览器接收进度');
    const detail = node('p', 'log-download-detail');
    const close = node('button', 'btn btn-sm', '关闭提示');
    close.type = 'button';
    close.hidden = true;
    close.onclick = () => {
      card.remove();
      if (panel.children.length === 1) { panel.remove(); panel = null; }
    };
    card.append(title, status, progress, detail, close);
    panel.append(card);
    panel.scrollTop = panel.scrollHeight;
    const started = Date.now();
    let received = false;
    const waiting = () => {
      detail.textContent = '已等待 ' + Math.floor((Date.now() - started) / 1000) + ' 秒' +
        (Number.isSafeInteger(expectedSize) ? ' · 文件 ' + size(expectedSize) : '') +
        '；板端读取并校验后开始接收，请保持页面打开。';
    };
    waiting();
    const timer = setInterval(() => { if (!received) waiting(); }, 1000);
    active.set(url, true);
    updateButtons(url);
    function finish(error) {
      clearInterval(timer);
      active.delete(url);
      updateButtons(url);
      close.hidden = false;
      card.classList.add(error ? 'download-failed' : 'download-saved');
      if (error) {
        progress.hidden = true;
        status.textContent = '下载失败';
        detail.textContent = error.message + '；可点击文件的下载按钮重试。';
      } else {
        progress.value = 100;
        status.textContent = snapshot ? '快照接收完成，已交给浏览器保存' : '接收完成，已交给浏览器保存';
        detail.textContent = '耗时 ' + ((Date.now() - started) / 1000).toFixed(1) +
          ' 秒 · ' + filename + '；请查看浏览器下载列表或保存提示。';
      }
    }
    try {
      const xhr = new XMLHttpRequest();
      xhr.open('GET', url);
      xhr.responseType = 'blob';
      xhr.onprogress = event => {
        // The server buffers and validates the SSH download before sending HTTP data.
        if (xhr.status !== 200) return;
        received = true;
        if (event.lengthComputable && event.total > 0) {
          const percent = Math.min(100, event.loaded / event.total * 100);
          progress.value = percent;
          status.textContent = '正在接收到浏览器 · ' + Math.floor(percent) + '%';
          detail.textContent = size(event.loaded) + ' / ' + size(event.total);
        } else {
          status.textContent = '正在接收到浏览器…';
          detail.textContent = '已接收 ' + size(event.loaded);
        }
      };
      xhr.onerror = () => finish(new Error('网络连接中断，请检查工作台服务和 SSH 连接'));
      xhr.onabort = () => finish(new Error('下载已中断'));
      xhr.onload = async () => {
        try {
          const blob = xhr.response;
          if (xhr.status !== 200) {
            let message = '下载请求失败（HTTP ' + xhr.status + '）';
            try { message = JSON.parse(await blob.text()).error || message; } catch (_) {}
            throw new Error(message);
          }
          if (!blob || (Number.isSafeInteger(expectedSize) && blob.size !== expectedSize))
            throw new Error('文件大小与列表不一致，请刷新文件列表后重试');
          const contentLength = xhr.getResponseHeader('Content-Length');
          if (contentLength !== null && blob.size !== Number(contentLength))
            throw new Error('文件接收不完整');
          // The server names bags using their recorded run, including UTF-8 Chinese.
          const disposition = xhr.getResponseHeader('Content-Disposition') || '';
          const encodedName = /filename\*=UTF-8''([^;]+)/i.exec(disposition);
          if (encodedName) {
            try {
              const suggested = decodeURIComponent(encodedName[1]);
              if (suggested && !/[\\/\x00-\x1f]/.test(suggested)) filename = suggested;
            } catch (_) {} // Keep the original name when a header is malformed.
          }
          const objectURL = URL.createObjectURL(blob);
          const link = node('a');
          link.href = objectURL;
          link.download = filename;
          document.body.append(link);
          try { link.click(); } finally {
            link.remove();
            setTimeout(() => URL.revokeObjectURL(objectURL), 60000);
          }
          finish();
        } catch (error) { finish(error); }
      };
      xhr.send();
    } catch (error) { finish(error); }
  }
  window.logDownloads = {
    bind(button, url, filename, expectedSize, displayName) {
      button.type = 'button';
      button.dataset.logDownloadUrl = url;
      button.dataset.logDownloadLabel = button.textContent;
      button.disabled = active.has(url);
      if (button.disabled) button.textContent = '下载中…';
      button.onclick = () => start(url, filename, expectedSize, displayName);
    }
  };
})();
