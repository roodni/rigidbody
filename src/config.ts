import * as scenes from './scene';


const sceneList: ((w: number) => scenes.Scene)[] = [
  (w) => new scenes.BodiesSchene(w),
  (w) => new scenes.LoopScene2(w),
  (w) => new scenes.LoopScene1(w),
  (w) => new scenes.PendulumScene(w),
  (w) => new scenes.BoxedWorldScene(w),
  (w) => new scenes.CollisionScene(),
];

const ShapeName = ['rectangle', 'circle'] as const;
type ShapeName = typeof ShapeName[number];

const config = {
  CANVAS_W: 500,
  CANVAS_H: 500,

  _scene: undefined as (scenes.Scene | undefined),
  setScene(idx: number) {
    this._scene = sceneList[idx](this.CANVAS_W);
    this._scene.init();
    return this._scene;
  },
  get scene() {
    let scene = this._scene;
    if (!scene) {
      scene = this.setScene(0);
    }
    return scene;
  },

  shapeName: ShapeName[0] as ShapeName,
  freeze: false,
};
export default config;


window.addEventListener('load', () => {
  // シーン切り替え
  const sceneSel = document.querySelector<HTMLSelectElement>('#scene')!;
  for (let i = 0; i < sceneList.length; i++) {
    const opt = document.createElement('option');
    opt.value = i.toString();
    opt.text = `scene ${i}`;
    sceneSel.add(opt);
  }
  sceneSel.addEventListener('change', () => {
    config.setScene(parseInt(sceneSel.value));
  });
  document.querySelector('#reset')!.addEventListener('click', () => {
    config.setScene(parseInt(sceneSel.value));
  });

  // 生成する図形の種類
  const shapeNameCntnr = document.querySelector('#shape_name_cntnr')!;
  for (const name of ShapeName) {
    const ipt = document.createElement('input');
    const lbl = document.createElement('label');
    ipt.type = 'radio';
    ipt.name = 'shape_name';
    if (name === config.shapeName) {
      ipt.checked = true;
    }
    ipt.addEventListener('change', () => {
      config.shapeName = name;
    });
    lbl.append(ipt, name);
    shapeNameCntnr.appendChild(lbl);
  }

  // フリーズ
  const freezeSel = document.querySelector<HTMLInputElement>('#freeze')!;
  freezeSel.checked = config.freeze;
  freezeSel.addEventListener('change', () => {
    config.freeze = freezeSel.checked;
  });
});