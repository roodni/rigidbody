import p5 from 'p5';
import * as scenes from './scene';

const sketch = (p: p5) => {
  const canvasW = 500;
  const canvasH = 500;

  // const scene = new scenes.LoopScene1(canvasW);
  const scene = new scenes.LoopScene2(canvasW);
  // const scene = new scenes.BodiesSchene(canvasW);
  // const scene = new scenes.PendulumScene(canvasW);

  p.setup = () => {
    p.createCanvas(canvasW, canvasH);
    scene.init(p);
  };

  p.draw = () => {
    scene.update(p);
  };
};

let myp5 = new p5(sketch);