# 拉取基础镜像
docker pull ubuntu:22.04
# 创建并进入一个新容器
docker run --name myenv -it ubuntu:22.04 bash
# 启动容器
docker start myenv
# 进入容器
docker exec -it myenv bash
# 删除容器
docker rm myenv
# 拷贝宿主机代码到容器里面
docker cp /home/youruser/uav_project myenv:/workspace
# 容器转为镜像
docker commit myenv myimage:1.0
# 镜像打包成压缩包（压缩包名可以随意起）
docker save -o myimage_1.0.tar myimage:1.0
# 在新设备上，从压缩包导入镜像，并从镜像创建进入容器
docker load -i myimage_1.0.tar
docker run --name myenv_new -it myimage:1.0 bash
