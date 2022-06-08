varying vec3 normal;
varying vec3 vertex;

const vec4 AMBIENT_BLACK = vec4(0.0, 0.0, 0.0, 1.0);
const vec4 DEFAULT_BLACK = vec4(0.0, 0.0, 0.0, 0.0);

uniform bool is_enabled[8];

bool IsLightEnabled(in int i)
{
    return is_enabled[i];
}

// i denotes the index of a light source.
/* bool IsLightEnabled(in int i)
{
    bool enabled = true;
   
    // If all the colors of the Light are set to BLACK then we know we do not need to bother
    // doing a lighting calculation on it.
    if ((gl_LightSource[i].ambient  == AMBIENT_BLACK) &&
        (gl_LightSource[i].diffuse  == DEFAULT_BLACK) &&
        (gl_LightSource[i].specular == DEFAULT_BLACK))
        enabled = false;
       
    return (enabled);
}
*/

// distance is measured from current vertex to the ith light source.
float CalculateAttenuation(in int i, in float distance)
{
    return(1.0 / (gl_LightSource[i].constantAttenuation +
                  gl_LightSource[i].linearAttenuation * distance +
                  gl_LightSource[i].quadraticAttenuation * distance * distance));
}

// N denotes the unit varying normal vector.
void DirectionalLight(in int i, in vec3 N, in float shininess,
                      inout vec4 ambient, inout vec4 diffuse, inout vec4 specular)
{
    vec3 L = normalize(gl_LightSource[i].position.xyz);
   
    float N_dot_L = dot(N, L);
   
    if (N_dot_L > 0.0)
    {   
        vec3 H = gl_LightSource[i].halfVector.xyz;
       
        float pf = pow(max(dot(N,H), 0.0), shininess);

        diffuse  += gl_LightSource[i].diffuse  * N_dot_L;
        specular += gl_LightSource[i].specular * pf;
    }
   
    ambient  += gl_LightSource[i].ambient;
}

// N denotes the unit varying normal vector, while V corresponds to the varying vertex position.
void PointLight(in int i, in vec3 N, in vec3 V, in float shininess,
                inout vec4 ambient, inout vec4 diffuse, inout vec4 specular)
{
    vec3 D = gl_LightSource[i].position.xyz - V;
    vec3 L = normalize(D);

    float distance = length(D);
    float attenuation = CalculateAttenuation(i, distance);

    float N_dot_L = dot(N,L);

    if (N_dot_L > 0.0)
    {   
        vec3 E = normalize(-V);
        vec3 R = reflect(-L, N);
       
        float pf = pow(max(dot(R,E), 0.0), shininess);

        diffuse  += gl_LightSource[i].diffuse  * attenuation * N_dot_L;
        specular += gl_LightSource[i].specular * attenuation * pf;
    }
   
    ambient  += gl_LightSource[i].ambient * attenuation;
}

// N denotes the unit varying normal vector, while V corresponds to the varying vertex position.
void Spotlight(in int i, in vec3 N, in vec3 V, in float shininess,
               inout vec4 ambient, inout vec4 diffuse, inout vec4 specular)
{
    vec3 D = gl_LightSource[i].position.xyz - V;
    vec3 L = normalize(D);

    float distance = length(D);
    float attenuation = CalculateAttenuation(i, distance);

    float N_dot_L = dot(N,L);

    if (N_dot_L > 0.0)
    {   
        float spot_effect = dot(normalize(gl_LightSource[i].spotDirection), -L);
       
        if (spot_effect > gl_LightSource[i].spotCosCutoff)
        {
            attenuation *=  pow(spot_effect, gl_LightSource[i].spotExponent);

            vec3 E = normalize(-V);
            vec3 R = reflect(-L, N);
       
            float pf = pow(max(dot(R,E), 0.0), shininess);

            diffuse  += gl_LightSource[i].diffuse  * attenuation * N_dot_L;
            specular += gl_LightSource[i].specular * attenuation * pf;
        }
    }
   
    ambient  += gl_LightSource[i].ambient * attenuation;
}

// N denotes the unit varying normal vector, while V corresponds to the varying vertex position.
void CalculateLighting(in int number_of_light_sources, in vec3 N, in vec3 V, in float shininess,
                       inout vec4 ambient, inout vec4 diffuse, inout vec4 specular)
{
    // Just loop through each light, and if its enabled add
    // its contributions to the color of the pixel.
    for (int i = 0; i < number_of_light_sources; i++)
    {
        if (IsLightEnabled(i))
        {
            if (gl_LightSource[i].position.w == 0.0)
                DirectionalLight(i, N, shininess, ambient, diffuse, specular);
            else if (gl_LightSource[i].spotCutoff == 180.0)
                PointLight(i, N, V, shininess, ambient, diffuse, specular);
            else
                Spotlight(i, N, V, shininess, ambient, diffuse, specular);
        }
    }
}

void main()
{
    // Normalize the normal. A varying variable CANNOT be modified by a fragment shader.
    // So a new variable needs to be created.
    vec3 n = normalize(normal);
   
    vec4 ambient, diffuse, specular, color;

    // Initialize the contributions for the front-face-pass over the lights.
    ambient  = vec4(0.0);
    diffuse  = vec4(0.0);
    specular = vec4(0.0);
   
    // In this case the built-in uniform gl_MaxLights is used to denote the number of lights.
    // A better option may be passing in the number of lights as a uniform or replacing the
    // current value with a smaller value.
    CalculateLighting(gl_MaxLights, n, vertex, gl_FrontMaterial.shininess,
                      ambient, diffuse, specular);
   
    color  = gl_FrontLightModelProduct.sceneColor  +
             (ambient  * gl_FrontMaterial.ambient) +
             (diffuse  * gl_FrontMaterial.diffuse) +
             (specular * gl_FrontMaterial.specular);

    // Re-initialize the contributions for the back-face-pass over the lights.
    ambient  = vec4(0.0);
    diffuse  = vec4(0.0);
    specular = vec4(0.0);
          
    // Now caculate the back contribution. All that needs to be done is to flip the normal.
    CalculateLighting(gl_MaxLights, -n, vertex, gl_BackMaterial.shininess,
                      ambient, diffuse, specular);

    color += gl_BackLightModelProduct.sceneColor  +
             (ambient  * gl_BackMaterial.ambient) +
             (diffuse  * gl_BackMaterial.diffuse) +
             (specular * gl_BackMaterial.specular);

    color = clamp(color, 0.0, 1.0);
   
    gl_FragColor = color;
}
