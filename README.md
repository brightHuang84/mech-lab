# 机械结构虚拟试验台 3D – 自由装配

Open `index.html` directly in a browser (works from file://; no build or server needed).
It is one self-contained file (about 5 MB) with Three.js r186, OrbitControls/TransformControls and Rapier 3D 0.21 (rapier3d-compat, WASM inlined as base64) bundled inside.

## Structure
- `index.html` – the built app (everything inlined)
- `build/src/parts.js` – part library: geometry (gear teeth, helix threads, lathe pulleys…), colliders, connection points, materials
- `build/src/physics.js` – Rapier world builder: rigid groups (parts joined by 固定连接 become one body), revolute/prismatic/spherical joints with exact local frames, motors, springs and dampers, a velocity-level coupling solver for gears/belts/chains/screws/gearbox/clutch, an edit-mode relaxation solver, and joint gap/energy measurement
- `build/src/presets.js` – 14 presets (incl. 三轮车), all built from the same free parts
- `build/src/app.js` – UI, scene, gizmo, tools, run/pause/reset/initial state, undo/redo, IO
- `build/template.html`, `build/build.sh` – to rebuild: `cd build && npm ci && ./build.sh`
- `tests/run-tests.js` – headless Chrome test suite (`node tests/run-tests.js`, 83 checks), results in `tests/test-results.txt`; `tests/assembly.js` (snap assembly) and `tests/joints.js` (joint editing, context menu, Tab, rotation, hand-built tricycle, preset connection audit) use real mouse/keyboard events

## 直接装配（自动连接）
拖动零件（从零件库拖入，或直接用鼠标拖动场景里的零件）靠近另一零件的连接点时，会显示预览：半透明零件、目标点圆圈和将要建立的连接类型（如“铰链”）。松开后零件精确对位，并自动建立连接（一次撤销即可撤回）。按住 **Alt**，或关闭工具栏上的“自动连接”，可以自由放置。

| 拖动的零件 → 目标 | 自动建立的连接 |
|---|---|
| 连杆端孔/中点 → 底座轴销、另一连杆孔、电机轴 | 铰链（电机轴上的铰链由电机驱动） |
| 轮/齿轮/带轮/链轮/飞轮/凸轮/蜗轮/锥齿轮中心孔 → 电机轴 / 底座轴销 / 连杆孔 | 铰链 |
| 同上中心孔 → 轴身（任意位置） | 固定在轴上 |
| 轴/丝杠/蜗杆端部 → 底座轴销 / 电机轴 | 铰链（轴承）/ 联轴（电机驱动） |
| 电机底部 → 底座/板/地面顶面 | 固定连接 |
| 球 → 连杆孔 | 固定连接 |
| 滑块 → 板/地面/底座顶面导轨 | 滑动副 |
| 螺母 → 丝杠 | 螺旋副（螺母滑动副 + 丝杠转轴，若无则自动加） |
| 直齿轮 → 直齿轮（同模数） | 按精确中心距和齿相位放置 + 啮合（没有转轴的齿轮自动装固定轴） |
| 齿条 ↔ 直齿轮 | 啮合 + 齿条滑动副 |
| 蜗杆 ↔ 蜗轮 | 啮合 |
| 锥齿轮 → 锥齿轮 | 90° 啮合（自动设置锥顶距） |

用鼠标拖动已装配的零件，它会按连接约束运动（绕铰链转动、沿导轨滑动）。用力拉开（超过约 12 cm）即可拆开，属性面板也有“拆开”和“▶ 试运行 3 秒”按钮。手动连接工具在左侧“高级”里。电机默认转速 30 rpm。

| 电机底部 / 轴承座底部 → 板、底座、连杆的面（任意位置，5 mm 网格） | 固定连接（电机/轴承座随车架一起运动） |
| 轴端 / 轴中心 → 轴承座轴孔 | 铰链 |
| 轮/齿轮中心孔 → 轴承座轴孔、转向叉轮轴 | 铰链 |
| 轴端、轮、连杆孔 → 板的侧面连接点（角、边中点、端面；数量可在板属性“每边侧面连接点数”设置） | 铰链 |
| 转向叉底部 → 板顶面 | 转向铰链（竖直销，带回正刚度和阻尼） |

新零件：**轴承座**（中心高 42.4 mm，与默认电机轴高一致）、**转向叉(前轮)**。默认轮半径改为 80 mm，方便用默认电机直接驱动。
自由（未固定）装配吸附后若有零件穿入地面，整个装配体会自动抬起到地面上。

## 修改连接
- **属性面板**：选中零件，“连接”列表列出它的每个连接；点击连接标记（彩色小标记，总是显示在最上层）也会打开该连接。每个连接可以：
  - 类型下拉框：铰链 / 滑动副 / 固定连接 / 球铰 / 螺旋副；传动（齿轮啮合、皮带等）有“啮合/启用”开关
  - 轴线：X / Y / Z / 自定义 + “反转”
  - “限位 / 驱动 / 摩擦”：角度或行程限位、铰链/滑动副驱动（rpm 或 mm/s + 最大扭矩/力）、与电机相连的铰链可开关“电机驱动”并改转速/方向、摩擦、阻尼、回正刚度、“设为转向销”
  - “删除连接”“拆开”（删除并把活动的一侧移开 3 cm）
  修改立即生效（编辑拖动、试运行马上体现），卡片标记“已修改”，每次修改都可撤销/重做。
- **右键菜单**（零件或连接标记上单击右键，不拖动）：改为铰链 / 改为固定 / 改为滑动 / 反转轴 / 删除连接 / 拆开 / 拆开全部连接 / 属性。
- **吸附时选择类型**：拖动中出现“松开 → 铰链   Tab: [铰链] / 固定连接 / 球铰”时按 **Tab**（Shift+Tab 反向）切换，松开即按所选类型连接。

## 旋转零件
- 拖动（零件库或场景中）时 **滚轮** 每格旋转 15°，绕当前视角方向的坐标轴；**Shift+滚轮 / Ctrl+滚轮** 切换旋转轴（视角 → X → Y → Z），标签显示轴和角度。带铰链的零件绕铰链转动。
- **中键按住零件拖动** = 自由旋转（15° 吸附，按住 Shift 精细）；带铰链的零件绕铰链转动、连接不脱开。中键拖空白处仍是相机操作。
- 吸附时自动转向：孔/轴的轴线自动对齐（轮→任意方向的轴、连杆→轴销、齿轮→倾斜的电机轴均已测试）。

## 手工搭三轮车（全部拖拽）
1. 拖入 **板**（车架）。
2. 拖入 **电机**，放到板顶面后部一侧（输出轴朝外）；再拖一个电机到另一侧，拖动时滚动滚轮转 180°（必要时 Shift+滚轮切到 Y 轴）让轴朝外。
3. 把 **轮** 拖到两个电机的输出轴上（铰链，电机驱动）。
4. 右键另一侧的轮 → **反转轴**（两侧电机方向相反，反转后同向前进）。
5. 拖入 **轴承座** 放到板前部顶面；把 **轴** 拖到轴承座轴孔；再把 **轮** 拖到轴上（固定在轴上）。
   或者：把 **转向叉(前轮)** 放到板前部顶面（转向铰链），再把轮拖到叉的轮轴上。
6. 按 ▶ 运行。也可直接载入示例“三轮车”。

## 示例连接审计
所有示例都经过“连接可视审计”测试：每个连接的锚点到两个零件实体的距离 < 1 mm（无悬空、无缝隙），除抛射物（石块）外每个活动零件都有连接。“简易小车”已重做：两台电机固定在车架上、输出轴插入后轮，前轮装在前轴上，前轴由两个固定在车架上的轴承座支承。

## 局限
- 螺旋副（作为连接类型）用速度级耦合实现（与齿轮等传动相同），长时间运行会有极小漂移。
- 平面安装面只在板/底座顶面和连杆正反面；板的底面不能直接安装。
- 转向叉是被动转向（带回正），没有方向盘/舵机控制；可用连接的“驱动”或“限位”锁定。
- 两个对置电机默认转向相反，需要对其中一个“反转轴”（三轮车示例已设置）。
- 右键菜单最多列出 5 个连接，更多请用属性面板。
