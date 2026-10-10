// node headless-test.js — 需要 puppeteer-core 与系统 Chrome
const puppeteer = require('/workspace/tools/node_modules/puppeteer-core');
(async () => {
  const browser = await puppeteer.launch({executablePath:'/usr/bin/google-chrome', headless:'new', args:['--no-sandbox','--disable-gpu']});
  const page = await browser.newPage();
  await page.setViewport({width:1440, height:1000, deviceScaleFactor:1});
  const errors = [];
  page.on('console', m => { if (['error','warning'].includes(m.type())) errors.push(m.type() + ': ' + m.text()); });
  page.on('pageerror', e => errors.push('pageerror: ' + e.message));
  await page.goto('file:///workspace/mech-lab/index.html');
  await new Promise(r => setTimeout(r, 800));
  const out = {};
  // 暂停在 θ=50° 便于截图
  await page.click('#bPlay');
  await page.evaluate(() => { const s = document.getElementById('angSlider'); s.value = 50; s.dispatchEvent(new Event('input')); });
  // 扫描
  await page.select('#swParam', 'l'); await page.click('#swRun');
  out.sweepMsg = await page.$eval('#swMsg', e => e.textContent);
  // 标定
  await page.click('.tabs button[data-tab=calib]');
  await page.click('#caDemo');
  out.calibBefore = await page.$eval('#caTiles', e => e.innerText.replace(/\n/g, ' '));
  await page.click('#caAuto');
  out.calibAfter = await page.$eval('#caTiles', e => e.innerText.replace(/\n/g, ' '));
  // 导出内容
  out.exp = await page.evaluate(() => {
    const M = window.__mech, csv = M.simCSV(1).split('\n'), svg = M.buildSVG(45);
    const doc = new DOMParser().parseFromString(svg, 'image/svg+xml');
    return {csvRows:csv.length - 1, csvHead:csv[0], csv90:csv[91], svgOk:!doc.querySelector('parsererror'), svgLen:svg.length, svgTexts:doc.querySelectorAll('text').length,
      bom:M.bomRows().length, json:Object.keys(M.designJSON())};
  });
  // 无效参数
  await page.evaluate(() => window.__mech.setParam('l', 50));
  out.invalidStatus = await page.$eval('#status', e => e.className + ' | ' + e.textContent);
  out.invalidTiles = await page.$eval('#lv_x', e => e.textContent);
  await new Promise(r => setTimeout(r, 300));
  await page.evaluate(() => window.__mech.setParam('l', 180));
  // 极端:l 略大于 r、高转速
  await page.evaluate(() => { window.__mech.setParam('r', 100); window.__mech.setParam('l', 101); window.__mech.setParam('rpm', 3000); });
  await new Promise(r => setTimeout(r, 300));
  out.extreme = await page.$eval('#status', e => e.className + ' | ' + e.textContent);
  await page.evaluate(() => { window.__mech.setParam('r', 60); window.__mech.setParam('l', 180); window.__mech.setParam('rpm', 60); });
  // 回到扫描页,截图
  await page.click('.tabs button[data-tab=sweep]');
  await new Promise(r => setTimeout(r, 500));
  await page.setViewport({width:1440, height:1500, deviceScaleFactor:1}); await page.evaluate(() => window.scrollTo(0,0)); await new Promise(r => setTimeout(r, 300));
  await page.addStyleTag({content:'aside{position:static!important;max-height:none!important}'}); await new Promise(r => setTimeout(r, 300));
  await page.screenshot({path:'/workspace/mech-lab/screenshot.png', fullPage:true});
  await page.setViewport({width:1440, height:1000, deviceScaleFactor:2}); await new Promise(r => setTimeout(r, 300));
  { const el = await page.$('#anim'); await el.screenshot({path:'/workspace/mech-lab/screenshot-anim.png'}); }
  await page.click('.tabs button[data-tab=calib]'); await new Promise(r => setTimeout(r, 300));
  await page.screenshot({path:'/workspace/mech-lab/screenshot-calib.png', fullPage:false});
  await page.click('.tabs button[data-tab=export]'); await new Promise(r => setTimeout(r, 300));
  out.svgPrev = await page.$eval('#svgPrev', e => e.innerHTML.length);
  await page.screenshot({path:'/workspace/mech-lab/screenshot-export.png', fullPage:true});
  console.log(JSON.stringify(out, null, 1));
  console.log('CONSOLE ERRORS/WARNINGS:', errors.length ? errors : 'none');
  await browser.close();
})();
