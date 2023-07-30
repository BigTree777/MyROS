# README

## Purpose
ROS2の勉強をするための個人リポジトリ.  

## Directory Architecture
my_ws: C++でROS2パッケージを作成するためのディレクトリ
  |- src
      |- hello_world: 単純なHelloWorld
      |- viewer_pointcloud: nuScenes形式の3D点群データを読み込んでPointCloud2のメッセージとしてトピックを送信するパッケージ

my_ws_python: PythonでROS2パッケージを作成するためのディレクトリ
    |- viewer_pointcloud: nuScenes形式の3D点群データを読み込んでPointCloud2のメッセージとしてトピックを送信するパッケージ
