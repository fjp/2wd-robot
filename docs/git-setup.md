## git setup

Install git on Ubuntu via the following command:

<pre><font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~/git/2wd-robot</b></font>$ sudo apt install git</pre>


Set your username and email address that you use on github (when using github to host your repository):

<pre><font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~</b></font>$ git config --global user.name &quot;fjp&quot;</pre>
<pre><font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~</b></font>$ git config --global user.email &quot;franz.pucher@gmail.com&quot;</pre>


To store your password credentials when pushing and pulling to the remote repository use the following commands:

<pre><font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~/git/2wd-robot</b></font>$ git config --global credential.helper store
<font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~/git/2wd-robot</b></font>$ git push
Username for &apos;https://github.com&apos;: fjp
Password for &apos;https://fjp@github.com&apos;: 
Everything up-to-date
<font color="#8AE234"><b>fjp@ubuntu</b></font>:<font color="#729FCF"><b>~/git/2wd-robot</b></font>$ git push
Everything up-to-date</pre>
