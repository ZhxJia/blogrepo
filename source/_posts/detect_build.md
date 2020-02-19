---
title: Apollo编译过程及存在的问题
categories:
- apollo
tags:
- build
mathjax: true
---

<!-- more -->





bazel 降级

> Latest Versions of Bazel Doesn't support git_repository (which is still used by tensorflow_hub), so Uninstalling Bazel 0.24.1 and installing Bazel 0.18.1 worked.

https://www.betaflare.com/3714.html