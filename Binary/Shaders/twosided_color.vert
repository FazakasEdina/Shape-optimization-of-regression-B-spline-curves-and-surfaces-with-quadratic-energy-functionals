varying vec3 normal;
varying vec3 vertex;
varying vec4 color_material;

void main()
{
	gl_FrontColor = gl_Color;
	gl_BackColor = gl_Color;
	color_material = gl_Color;
    // Calculate the normal
    normal = normalize(gl_NormalMatrix * gl_Normal);
   
    // Transform the vertex position to eye space
    vertex = vec3(gl_ModelViewMatrix * gl_Vertex);
       
    gl_Position = ftransform();
}