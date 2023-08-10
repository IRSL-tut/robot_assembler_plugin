# robot_assembler_plugin
robot_assembler_plugin for choreonoid

Choreonoid version of https://github.com/agent-system/robot_assembler

This project would be compatible with robot_assembler.

## compile with choreonoid
Add this repository to choreonoid/ext directory

After that, run cmake and make install in choreonoid main directory

## run sample
choreonoid --assembler config/kxr/kxr_assembler_config.yaml config/assembler.cnoid

## 操作方法

[RobotAssemblerの使い方](https://github.com/IRSL-tut/CPS-lecture/wiki/RobotAssembler%E3%81%AE%E4%BD%BF%E3%81%84%E6%96%B9)

## ソースファイル説明
- src/RobotAssemblerPlugin.{cpp, h}
  - choreonoidプラグイン登録
  - Viewの登録やコマンドラインオプションのパースを行う

- src/RobotAssembler.{cpp, h}
  - ロボット構成のグラフ構造を記述
  - euslispのcascaded-coordsに相当する

- src/RobotAssemblerSettings.{cpp, h}
  - settings.yamlや.roboasmの読み込み/書き出しを記述
  - yamlファイル形式を用いて、choreonoidのYAMLReader/YAMLWriterを使用する

- src/RobotAssemblerBody.{cpp, h}
  - ロボット構成のグラフ構造を choreonoid.body への変換を記述

- src/RobotAssemblerHelper.{cpp, h}
  - ロボット構成のグラフ構造を choreonoidで表示可能なSceneGraph への変換を記述
  - 表示したロボットのクリック等のイベントの記述

- src/AssemblerManager.{cpp, h}
  - GUIを用いたAssemblerの操作を行う核となるAssemblerManagerを記述
  - イベント等の処理、アイテムの取扱、ビュー等への表示指示等はAssemblerManagerを介して行う

- src/AssemblerItem.{cpp, h}
  - 表示可能Itemとしての取扱を記述

- src/AssemblerView.{cpp, h}
  - パーツの追加等を行うパネルについて記述

- src/AssemblerBar.{cpp, h}
  - ツールバーについて記述

- src/AssemblerTreeView.{cpp, h}
  - ツリー構造表示を行うパネルについて記述

- src/AssemblerPartsView.{cpp, h}
  - パーツ情報表示を行うパネルについて記述 

- src/irsl_choreonoid/Coordinates.h
  - euslisp coordinatesクラス互換のクラス定義

- src/irsl_debug.h
  - デバッグ表示用のマクロ定義

- src/exportdecl.h
  - export用のマクロ定義

