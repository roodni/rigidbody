import p5 from 'p5';
import * as scenes from './scene';

const sketch = (p: p5) => {
  const canvasW = 500;
  const canvasH = 500;

  const scene = new scenes.LoopWorldScene(canvasW);
  // const scene = new scenes.MySchene(canvasW);

  p.setup = () => {
    p.createCanvas(canvasW, canvasH);
    scene.init(p);
  };

  p.draw = () => {
    scene.update(p);
  };
};

let myp5 = new p5(sketch);