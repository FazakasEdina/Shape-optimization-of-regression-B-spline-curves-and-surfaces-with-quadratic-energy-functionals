/*
It is assumed that the OpenGL 2.0 application provides:
        * projection and model view matrices;
        * several light sources using light indices GL_LIGHT0, GL_LIGHT1,..., GL_LIGHT7;
        * materials for front and back faces; and
        * a normal vector for each vertex.
*/

varying vec3 normal;
varying vec3 vertex;

void main()
{
    // calculate and normalize the normal vector
    normal = normalize(gl_NormalMatrix * gl_Normal);
   
    // transform the vertex position to eye space
    vertex = vec3(gl_ModelViewMatrix * gl_Vertex);
       
    gl_Position = ftransform();
}
