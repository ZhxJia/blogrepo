---
# ！！！！！！！！！！
# 每一项的 : 后面均有一个空格
# 且 : 为英文符号
# ！！！！！！！！！！

title:
# 文章标题，可以为中文

date:
# 建立日期，如果自己手动添加，请按固定格式
# 就算不写，页面每篇文章顶部的发表于……也能显示
# 只要在主题配置文件中，配置了 created_at 就行
# 那为什么还要自己加上？
# 自定义文章发布的时间

updated:
# 更新日期，其它与上面的建立日期类似
# 不过在页面每篇文章顶部，是更新于……
# 在主题配置文件中，是 updated_at

permalink:
# 若站点配置文件下的 permalink 配置了 title
# 则可以替换文章 URL 里面的 title（文章标题）

categories:
# 分类，支持多级，比如：
# - technology
# - computer
# - computer-aided-art
# 则为 technology/computer/computer-aided-art
# （不适用于 layout: page）

tags:
# 标签
# 多个可以这样写 [标签1,标签2,标签3]
# （不适用于 layout: page）

mathjax:
# latex 公式 true

description:
# 文章的描述，在每篇文章标题下方显示
# 并且作为网页的 description 元数据
# 如果不写，则自动取 <!-- more -->
# 之前的文字作为网页的 description 元数据

keywords:
# 关键字，并且作为网页的 keywords 元数据
# 如果不写，则自动取 tags 里的项
# 作为网页的 keywords 元数据

comments:
# 是否开启评论
# 默认值是 true
# 要关闭写 false


link:
# 文章的外部链接
# https://io-oi.me/tech/hello-world/


top:
# 文章置顶
# 此项只有参考本文 5.15 节配置好，否则请勿添加！

#<!-- more -->
---
