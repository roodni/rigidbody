import p5 from 'p5';
import * as scenes from './scene';

const sceneList: (() => scenes.Scene)[] = [
  () => new scenes.LoopScene2(canvasW),
  () => new scenes.LoopScene1(canvasW),
  () => new scenes.BodiesSchene(canvasW),
  () => new scenes.PendulumScene(canvasW),
  () => new scenes.BoxedWorldScene(canvasW),
  () => new scenes.CollisionScene(),
];

const canvasW = 500;
const canvasH = 500;

let scene: scenes.Scene;
function loadScene(idx: number) {
  scene = sceneList[idx]();
  scene.init();
}
loadScene(0);

const sketch = (p: p5) => {
  p.setup = () => {
    const cvs = p.createCanvas(canvasW, canvasH);
    cvs.parent('cvs')
  };

  p.draw = () => {
    scene.update(p);
  };

  p.mousePressed = () => {
    scene.mousePressed(p);
  };
  p.mouseReleased = () => {
    scene.mouseReleased(p);
  };
};


let myp5 = new p5(sketch);

window.addEventListener('load', () => {
  // シーン切り替え
  const sceneSel = document.querySelector<HTMLSelectElement>('#scene')!;
  for (let i = 0; i < sceneList.length; i++) {
    const opt = document.createElement('option');
    opt.value = i.toString();
    opt.text = i.toString();
    sceneSel.add(opt);
  }
  sceneSel.addEventListener('change', () => {
    loadScene(parseInt(sceneSel.value));
  });
  document.querySelector('#reload')?.addEventListener('click', () => {
    loadScene(parseInt(sceneSel.value));
  });


});