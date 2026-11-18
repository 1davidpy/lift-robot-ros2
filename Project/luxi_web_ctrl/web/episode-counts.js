/* Counts come from finalized raw manifests on the receiver, not the RAM queue. */
(() => {
  const fields = ['accepted', 'failed', 'cancelled', 'total'];
  const set = (id, value) => { document.getElementById(id).textContent = value; };
  function renderReceiverCounts(receiver) {
    const counts = receiver.peer?.episode_counts;
    const valid = receiver.online && counts?.valid === true &&
      [...fields, 'unknown'].every(key => Number.isSafeInteger(counts[key]) && counts[key] >= 0) &&
      counts.total === counts.accepted + counts.failed + counts.cancelled + counts.unknown;
    fields.forEach(key => set('episode-count-' + key, valid ? counts[key] : '—'));
    set('episode-counts-status', valid ? '已更新' : '统计暂不可用');
    set('episode-counts-note', valid
      ? `保存目录：${receiver.peer.directory} · 约每 4 秒更新；录制中和传输中暂不计入。${counts.unknown ? `另有 ${counts.unknown} 条标签异常，请检查。` : ''}`
      : '等待接收电脑在线并报告统计；连接恢复后自动更新，不将未知数量显示为零。');
  }
  async function pollCounts() {
    try {
      const response = await fetch('/api/receiver', {cache: 'no-store', signal: AbortSignal.timeout(3000)});
      if (!response.ok) throw new Error('counts unavailable');
      renderReceiverCounts(await response.json());
    } catch { renderReceiverCounts({online: false}); }
    setTimeout(pollCounts, 4000);
  }
  pollCounts();
})();
