## 一、常用Git命令速查表
# 初始化仓库
git init
# 克隆远程仓库
git clone <仓库URL>
  # 查看当前状态
  git status
  # 添加文件到暂存区
  git add <文件名>      # 添加单个文件
    git add .            # 添加所有修改
    # 提交更改
    git commit -m "提交说明"
    ## 二、分支管理
    # 创建分支
    git branch <分支名>
      # 切换分支
      git checkout <分支名>
        git switch <分支名>    # Git 2.23+推荐
          # 创建并切换分支
          git checkout -b <新分支名>
            # 合并分支
            git merge <目标分支>
              # 删除分支
              git branch -d <分支名>
                ##三、撤销操作
                # 撤销工作区修改
                git checkout -- <文件名>
                  # 撤销暂存区文件
                  git reset HEAD <文件名>
                    # 修改最后一次提交
                    git commit --amend
                    ##四、远程仓库操作
                    # 添加远程仓库
                      git remote add origin <仓库URL>
                      # 推送本地分支
                      git push -u origin <分支名>
                        # 拉取远程更新
                        git pull origin <分支名>
                          # 获取远程分支
                          git fetch origin