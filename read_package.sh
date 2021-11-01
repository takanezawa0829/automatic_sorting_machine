cd ../../
echo "catkin_makeを実行します。"
catkin_make
cd src
cd automatic_sorting_machine
cd scripts/
echo "ファイルに実行権限を付与します。"
chmod +x *
cd ..
echo "完了しました。"