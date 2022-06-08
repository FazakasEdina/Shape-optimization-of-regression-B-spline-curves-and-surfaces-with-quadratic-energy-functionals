-----------------------------------------
About the OpenGL Extension Library (GLEW)
-----------------------------------------
Although these dynamically and statically linkable function 
libraries are named as glew32.dll and glew32.lib, respectively, 
they have been generated for 64-bit Windows platforms, by using 
the compiler of the Community Edition of the Microsoft Visual 
Studio 2019. 

At run-time, your debug/release 64-bit application will try to 
connect to the 64-bit version of the file glew32.dll (for some 
reason, this confusing naming convention has been applied by 
the developers of the OpenGL Extension Wrangler Library). 

Therefore, please do not rename these files, e.g., to glew64.dll 
and glew64.lib, respectively (otherwise your application will 
crash at its start-up).