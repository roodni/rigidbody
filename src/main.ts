import type p5 from 'p5';
import config from './config';

const LAPTIMES_NUM = 60;
const laptimes: number[] = [];

(async () => {
  const p5 = (await import('p5')).default;

  const sketch = (p: p5) => {
    p.setup = () => {
      const cvs = p.createCanvas(config.CANVAS_W, config.CANVAS_H);
      cvs.parent('cvs')
      p.strokeWeight(2)
    };

    p.draw = () => {
      config.scene.update(p);

      const tm = performance.now();
      if (laptimes.length > LAPTIMES_NUM) {
        laptimes.shift();
      }
      if (laptimes.length >= 1) {
        const fps = Math.round(laptimes.length / (tm - laptimes[0]) * 1000);
        document.querySelector('#fps')!.innerHTML = fps.toString();
      }
      laptimes.push(tm);
    };

    p.mousePressed = () => {
      config.scene.mousePressed(p);
    };
    p.mouseReleased = () => {
      config.scene.mouseReleased(p);
    };
  };

  let myp5 = new p5(sketch);
})();