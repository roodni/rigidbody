import p5 from 'p5';
import config from './config';


const sketch = (p: p5) => {
  p.setup = () => {
    const cvs = p.createCanvas(config.CANVAS_W, config.CANVAS_H);
    cvs.parent('cvs')
    p.strokeWeight(2)
  };

  p.draw = () => {
    config.scene.update(p);
  };

  p.mousePressed = () => {
    config.scene.mousePressed(p);
  };
  p.mouseReleased = () => {
    config.scene.mouseReleased(p);
  };
};

let myp5 = new p5(sketch);