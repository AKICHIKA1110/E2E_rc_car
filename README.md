# Raspberry PiにJupyter Labを自動起動する方法

ステップ 1: JupyterLab のインストール
Raspberry Pi に JupyterLab を以下のコマンドでインストール
```
pip install jupyterlab
```

ステップ 2: JupyterLab の起動設定ファイルの作成
Raspberry Pi が起動するたびに自動的に JupyterLab を起動するためのシステムサービスを設定します。
システムサービスファイルの作成
以下のコマンドを実行して、jupyterlab.service という新しいサービスファイルを作成します。
```
sudo nano /etc/systemd/system/jupyterlab.service
```

サービスファイルに以下の内容を記述
jupyterlab.service ファイルに以下の内容を追加します。/home/pi/ はユーザーのホームディレクトリに置き換えてください。
```jupyterlab.service
[Unit]
Description=JupyterLab
After=network.target

[Service]
Type=simple
User=pi  # Raspberry Pi のユーザー名
ExecStart=/usr/bin/python3 -m jupyter lab --ip=0.0.0.0 --port=8888 --no-browser --notebook-dir=/home/pi/notebooks
WorkingDirectory=/home/pi
Environment="PATH=/home/pi/.local/bin:/usr/bin:/bin"
Restart=always

[Install]
WantedBy=multi-user.target
```
--ip=0.0.0.0: Raspberry Piの全てのIPアドレスでJupyterLabにアクセス可能にします。
--port=8888: JupyterLabがポート8888で起動されるように指定しています。
--no-browser: ブラウザを自動的に開かないように設定します（Raspberry Pi上では必要ないため）。
--notebook-dir=/home/pi/notebooks: JupyterLabのルートディレクトリを指定します。このディレクトリを変更したい場合は、任意のパスに変更してください。
ファイルを保存してエディタを終了します。Ctrl + X、Y、Enter で保存してエディタを閉じます。

サービスを有効化
以下のコマンドを実行して、作成したサービスを有効化します。

bash
コードをコピーする
sudo systemctl enable jupyterlab
サービスを開始
サービスを手動で開始して、動作確認を行います。

bash
コードをコピーする
sudo systemctl start jupyterlab
ステータスの確認
JupyterLab が正しく起動しているかを確認するため、以下のコマンドを使用します。

bash
コードをコピーする
sudo systemctl status jupyterlab
ここで、サービスが active (running) となっていることを確認します。

ステップ 4: 自動起動の確認
Raspberry Pi を再起動して、起動後に http://192.168.11.2:8888 にアクセスできるか確認してください。再起動は以下のコマンドで行えます。

bash
コードをコピーする
sudo reboot
再起動後、ブラウザで http://192.168.11.2:8888 にアクセスすると、JupyterLab が自動的に起動されているはずです。
