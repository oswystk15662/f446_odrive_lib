ブランチ名はミスった。matsu-versionがmain

# f446_odrive_base

# f446_odrive_can
仕様

odrive_callback()で、heart beat msg or iq msgを受け取る。
両方受け取れなかったら、idleにして、人に緊急停止を押させる。printfでマイコン落としてもいいけど、どう動くかわからないので。
