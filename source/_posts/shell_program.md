---

title: shell脚本编程基础
categories:
- shell
tags:
- shell
mathjax: true
---

shell 脚本可以让计算机自动完成一系列的工作，通过解释运行。

<!--more-->

[TOC]

shell脚本的第一行一般是`#! /bin/bash` 告诉系统这个脚本需要用什么解释器来执行，即用哪一种shell,`#!`是约定的标记
运行shell脚本的两种方法：

作为可执行程序(通过`./`执行要求文件有执行权限)

```bash
chmod +x ./test.sh #使脚本具有执行权限
./test.sh # 执行脚本
```

作为解释器参数(文件可以无执行权限)

```bash
bash test.sh
```

`source`命令作用是在当前bash环境下读取并执行文件中的命令。

```
source test.sh
```



## 1. Shell编程语法

### 1.1 Shell 变量

**定义变量：**
	定义变量时，不需要加`$`符号，且变量名和等号之间不能有空格，通过字母数字下划线组成，开头不能为数字：

```bash
my_name="jac"
```

**使用变量：**
	使用一个定义过的变量，只需要在变量名之前加`$`,如：

```bash
my_name="jac"
echo $my_name
echo ${my_name}
```

变量名外的花括号可选，花括号用于帮助解释器识别变量的边界。
**只读变量：**
	使用`readonly`命令可以将变量定义为只读变量：

```bash
my_name="jac"
readonly my_name
```

**删除变量：**
	使用`unset`命令可以删除变量：

```bash
unset my_name
```

**变量类型：**
	运行shell时，会同时存在三种变量：
		(1) 局部变量：在脚本或命令中定义，仅在当前shell实例中有效，其他shell启动的程序不能访问局部变量
		(2) 环境变量：所有的程序，包括shell启动的程序，都能访问环境变量，有些程序需要环境变量来保证其正常运行，		必要的时候shell脚本也可以定义环境变量。
		(3) shell变量：shell变量时shell 程序设置的特殊变量，shell变量中有一部分是环境变量，有一部分是局部变量。

### 1.2 shell 变量类型

**shell字符串:**
	字符串是shell编程中最常用的数据类型，可以用单引号，也可以用双引号表示：

```bash
str='this is a string'
```

单引号字符串的限制：
	单引号里的任何字符都会原样输出，且单引号字符串中的变量,转义符都是是无效的。

```bash
my_name='jac'
str="Hello,${my_name}"
```

双引号字符串的优点：
	双引号里可以出现转义字符，也可以含有变量。

拼接字符串：

```bash
your_name='jachin'
my_name='jac'
echo $your_name $my_name
```

获取字符串长度：

```bash
my_name="jachin"
echo ${#my_name} #输出4
```


提取子字符串：

```bash
my_name="jachin"
echo ${my_name:1:4} #输出achi
```

​	字符串截取方式：`#,##`表示从左边开始删除，一个`#`表示从左边删除到第一个指定的字符，两个`#`表示从左边开始删除到最后一个指定的字符，而`%,%%`表示从右边开始删除，一个`%`表示从右边删除到第一个指定的字符，两个`%`表示从右边删除到最后一个指定的字符。

查找子字符串：查找字符h或i的位置（谁先出现，就返回谁）

```bash
my_name="jachin"
echo `expr index "$my_name" io`
```

注意是反引号 ` 不是单引号 ’

**Shell数组：**
bash支持一维数组（不支持多维数组），并且没有限定数组的大小,与C语言类似，数组元素的下标从0开始编号，并通过下标获取数组中的元素。

定义数组：

```bash
array_name=(value0 value1 value2 value3)
```

读取数组：

```bash
value=${array_name[n]}
value=${array_name[@]}
```

使用`@`可以获取数组中的所有元素
获取数组长度：

```bash
length=${#array_name[@]}
```

### 1.3 shell 传递参数

在执行脚本时，可以脚本传递参数，脚本内获得参数的格式为：`$n`,n代表一个数字，1为执行脚本的第一个参数，2为执行脚本的第二个参数，同时还有一下特殊字符`#,*,$,!,@,-,?`

| 参数 |                            说明                             |
| :--: | :---------------------------------------------------------: |
| `$#` |                   传递到的脚本的参数个数                    |
| `$*` |           以一个单字符串显示所有向脚本传递的参数            |
| `$$` |                   脚本运行的当前进程id号                    |
| `$!` |                后台运行的最后一个进程的id号                 |
| `$@` |           以一个单字符串显示所有向脚本传递的参数            |
| `$-` |                   显示shell使用的当前选项                   |
| `$?` | 显示最后命令的退出状态，0表示没有错误，其他任何值表明有错误 |

`$*`当以双引号" "括起来，则返回的是"\$1,\$2,..."
`$@`当以双引号" "括起来，则返回的是"\$1","\$2",...

```bash
bash test.sh 1 2 3
---
for i in "$*";do
	echo $i
done
#输出 1 2 3
for i in "$@";do
	echo $i
done
#输出 
1
2
3
```



### 1.4 Shell 运算符

shell支持多种运算符，包括：

- 算数运算符
- 关系运算符
- 布尔运算符
- 字符串运算符
- 文件测试运算符

原生的bash不支持数学运算，但是可以通过`awk` 和`expr`等命令来实现。`expr`较常用，其是表达式计算工具，使用它能够完成表达式的求值操作。

```bash
#!/bin/bash

val=`expr 2 + 2`
```

注意表达式和运算符之间要有空格，且表达式要被``包含。

**算数运算符：**

| 运算符 | 说明                                       | 举例                |
| :----- | ------------------------------------------ | ------------------- |
| +      | 加法                                       | \`expr \$a + $b\`   |
| -      | 减法                                       | \`expr \$a - $b\`   |
| *      | 乘法                                       | \`expr \$a \\* $b\` |
| /      | 除法                                       | \`expr \$a / $b\`   |
| %      | 取余                                       | \`expr \$a % $b\`   |
| =      | 赋值                                       | a=$b                |
| ==     | 相等。用于比较两个数字，相同则返回True     | [ $a == \$b ]       |
| !=     | 不相等。用于比较两个数字，不相同则返回True | [ $a != \$b ]       |

注意：运算符左右的空格,以及[  ]与变量之间的空格, \`expr 10 + 20\`可替换为$(expr 10 + 20)

**关系运算符：**

| 运算符 | 说明                                         | 举例           |
| :----- | -------------------------------------------- | -------------- |
| -eq    | 检查两个数是否相等，相等则返回true           | [ $a -eq \$b ] |
| -ne    | 检查两个数是否不相等，不相等则返回true       | [ $a -ne \$b ] |
| -gt    | 检查左边的数是否大于右边的，是则返回true     | [ $a -gt \$b ] |
| -lt    | 检查左边的数是否小于右边的，是则返回true     | [ $a -lt \$b ] |
| -ge    | 检查左边的数是否大于等于右边的，是则返回true | [ $a -ge \$b ] |
| -le    | 检查左边的数是否小于等于右边的，是则返回true | [ $a -le \$b ] |

注：可用于条件语句：if.. then .. fi
		eq->equal ,ne->not equal ,gt->greater than,lt->less than,ge->greater than or equal ,le->less than or equal

**布尔运算符：**

| 运算符 | 说明                                   | 举例                         |
| ------ | -------------------------------------- | ---------------------------- |
| !      | 非运算，表达式为true则返回false        | [ !false ] 返回true          |
| -o     | 或运算，有一个表达式为true，则返回true | [ $a -lt 20 -o \$b -gt 100 ] |
| -a     | 与运算，两个表达式都为true才返回true   | [ $a -lt 20 -a \$b -gt 100 ] |

**逻辑运算符：**

| 运算符 | 说明      | 举例                            |
| ------ | --------- | ------------------------------- |
| &&     | 逻辑的AND | [[ $a -lt 100 && \$b -gt 100 ]] |
| \|\|   | 逻辑的OR  | [[ $a -lt 100 || \$b -gt 100 ]] |

注意：逻辑运算用的是[[ ... ]] ,[[ ... ]]是[ ... ]运算符的补充，支持逻辑运算符||,&&，不再使用 -a,-o

```bash
#!/bin/bash

a=10
b=20
if [ $a -lt 100 -a $b -gt 100 ]
then
   echo "返回 true"
else
   echo "返回 false" #bingo
fi
```

**字符串运算符：**

| 运算符 | 说明                                   | 举例         |
| ------ | -------------------------------------- | ------------ |
| =      | 检测两个字符串是否相等，相等返回true   | [ $a = \$b ] |
| !=     | 检测两个字符串是否相等，不相等返回true | [ $a != \$b] |
| -z     | 检测字符串长度是否为0,为0返回true      | [ -z $a ]    |
| -n     | 检测字符串长度是否不为0,不为0返回true  | [ -n "$a"]   |
| $      | 检测字符串是否为空，不为空返回true     | [ $a ]       |

```bash
#!/bin/bash
a='abcdefg'
if [ -n "$a" ]
then
   echo "-n $a : 字符串长度不为 0"
else
   echo "-n $a : 字符串长度为 0" #bingo
fi
if [ $a ]
then
   echo "$a : 字符串不为空" #bingo
else
   echo "$a : 字符串为空"
fi
```

注意：判断字符串长度是否为0，应该[ -n "$a" ]而不是[ -n \$a ]

**文件测试运算符：**

| 操作符  | 说明                                                   | 举例         |
| ------- | ------------------------------------------------------ | ------------ |
| -b file | 检测文件是否是块设备文件，是则返回true                 | [ -b $file ] |
| -c file | 检测文件是否是字符设备文件，是则返回true               | [ -c $file ] |
| -d file | 检测文件是否是目录，是则返回true                       | [ -d $file ] |
| -f file | 检测文件是否是普通文件，是则返回true                   | [ -f $file ] |
| -g file | 检测文件是否设置了SGID位，如果是则返回true             | [ -g $file ] |
| -k file | 检测文件是否设置了粘着位(sticky bit)，如果是则返回true | [ -k $file ] |
| -p file | 检测文件是否是有名管道，如果是则返回true               | [ -p $file ] |
| -u file | 检测文件是否设置了SUID位，如果是则返回true             | [ -u $file ] |
| -r file | 检测文件是否可读，如果是，则返回true                   | [ -r $file ] |
| -w file | 检测文件是否可写，如果是，则返回true                   | [ -w $file ] |
| -x file | 检测文件是否可执行，如果是，则返回true                 | [ -x $file ] |
| -s file | 检测文件是否位空（文件大小是否大于0），不为空返回true  | [ -s $file ] |
| -e file | 检测文件（包括目录）是否存在，如果是，则返回true       | [ -e $file ] |

```bash
file="/var/www/runoob/test.sh"
if [ -e $file ]
then
   echo "文件存在"
else
   echo "文件不存在"
fi
```

### 1.5 Shell 指令

**echo指令：**
用于字符串的输出，命令格式为：

```bash
echo string
```

若要显示转义字符：需要用反斜线 `\`

```bash
echo "\"It is a test\""
```

使用`-e`可开始转义：

```bash
echo -e "OK! \n" #换行
echo "It is a test"
```

```
OK!

It is  a test
```

```bash
echo -e "OK! \c" #不换行
echo "It is a test"
```

```
OK! It is a test
```

若要显示变量，则双引号内 $name ,若要原样输出，则用单引号

```bash
#!/bin/sh
read name 
echo "$name It is a test"
```

注意：`read`命令从标准输入读取一行，各个参数通过空格分隔，

若要将显示结果定向至文件，则：

```bash
echo "It is a test" > myfile
```

注意：重定向：`>>` 重定向追加，在原文件末尾追加内容，`>`重定向输出，替换原有文件内容

显示命令执行结果：

```bash
echo `date`
```

注意这里用的是反引号(ESC键下方)，不是单引号

**printf命令：**
`printf`命令模仿c程序中的printf()函数,与echo相比，`printf`需要手动添加换行，其命令语法：

```bash
printf format-string [arguments...]
```

`format-string`:为格式控制字符串
`arguments`:为输出参数列表

```bash
%d %s %c %f 格式替代符详解:
d: Decimal 十进制整数 -- 对应位置参数必须是十进制整数，否则报错！
s: String 字符串 -- 对应位置参数必须是字符串或者字符型，否则报错！
c: Char 字符 -- 对应位置参数必须是字符串或者字符型，否则报错！
f: Float 浮点 -- 对应位置参数必须是数字型，否则报错！

printf "%d %s %c\n" 1 "abc" "def"
# 1 abc d    %c会自动截取字符串的第一个字符
```

**shell test命令：**
`test`命令用于检查某个条件是否成立，它可以进行数值，字符和文件三个方面的测试。

```bash
# 文件夹不存在则创建
if [ ! -d "/data/" ];then
  mkdir /data
  else
  echo "文件夹已经存在"
fi
```



### 1.6 Shell 流程控制

需要注意的是shell的流程控制不可为空

**if else**
语法格式：

```bash
if condition:
then
	command1
	command2
	..
	commandN
else
	command
fi
```

写成一行

```bash
if [ ... ]; then command ; fi
```

注意`;`后的空格。

```bash
if condition1
then
	command1
elif condition2
then 
	command2
else 
	commnadN
fi
```

**for**
for循环的一般格式为：

```bash
for var in item1 item2 ... itemN
do
	command1
	command2
	...
	commandN
done
```

写成一行：

```bash
for var in item1 ... itemN; do command1; cammand2... done;
```

也可表示为,与c语言for循环类似的形式

```bash
for((assignment;condition;next));do
    command_1;
    command_2;
    commond_..;
done;
```

```bash
#!/bin/bash
for((i=1;i<=5;i++));do
    echo "这是第 $i 次调用";
done;
```



**while**
while循环用于不断执行一系列命令，也用于从文件中读取数据，其一般格式为：

```bash
while condition
do
	command
done
```

```bash
#!/bin/bash
int=1
while(( $int<=5 ))
do
    echo $int
    let "int++"
done
```

其中 `let`用于执行表达式，变量计算中不需要`$`来表示变量，也可以省略 " " ,例如 `let n++` 实现自加，同时`let`与
`expr`相比不需要用空格隔开各个运算的字符

```bash
# 循环读取
echo '按下 <CTRL-D> 退出'
echo -n '输入你最喜欢的网站名: '
while read FILM
do
    echo "是的！$FILM 是一个好网站"
done
```

**until**
until循环执行一系列指令直至条件为true时停止，until循环与while循环在处理方式时刚好相反：

```bash
until condition #返回值为false,则继续执行
do
    command
done
```

**case**
case语句匹配一个值与一个模式：

```bash
case val in
模式1)
	command1
	...
	commandN
	;;
模式2)
	command1
	...
	commandN
	;;
esac #倒序表示结束
```

每一模式以右括号`)`结束，取值可以为变量或常数，当val与模式匹配后，执行模式到`;;`结束，其他模式将不再执行。

**break**
break允许跳出所有循环（终止执行该循环）

**continue**
continue命令与break命令相似，但是continue不会跳出所有循环，仅仅跳出当前循环。

### 1.7 Shell 函数

shell脚本中用户可以定义函数，然后在shell脚本中可以随便调用：
格式如下：

```bash
[ function ] funname [()]

{

    action;

    [return int;]

}
```

说明：可以function fun()定义，也可以直接fun()定义，不带任何参数；参数返回可以显示加 `return`,如果不加，将以最后一条命令运行结果，作为返回值。

```bash
funWithReturn(){
    echo "这个函数会对输入的两个数字进行相加运算..."
    echo "输入第一个数字: "
    read aNum
    echo "输入第二个数字: "
    read anotherNum
    echo "两个数字分别为 $aNum 和 $anotherNum !"
    return $(($aNum+$anotherNum))
}
funWithReturn
echo "输入的两个数字之和为 $? !"
```

函数返回值在调用该函数后，通过`$?`来获得,且其只对上一条指令负责，一旦函数返回后其返回值没有立即保存到参数，那么其返回值将不再能通过`$?`获得。
注意：所有函数在使用前必须定义，即一般函数放在脚本开始的部分，调用函数时仅使用函数名即可。

**函数参数**

在shell中，调用函数时，可以向其传递参数，在函数体内部，通过`$n`的形式来获取参数的值，例如`$1`表示第一个参数。
当n>=10时，需要使用`${n}`来获取参数，`${10}`

```bash
funWithParam(){
    echo "第一个参数为 $1 !"
    echo "第二个参数为 $2 !"
    echo "第十个参数为 $10 !"
    echo "第十个参数为 ${10} !"
    echo "第十一个参数为 ${11} !"
    echo "参数总数有 $# 个!"
    echo "作为一个字符串输出所有参数 $* !"
}
funWithParam 1 2 3 4 5 6 7 8 9 34 73
```

注意：函数或命令的执行结果可以作为条件语句，须注意的是，shell语言中函数正确执行的结果0代表true