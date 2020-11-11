/* Fragment shader
 */

 varying vec3 normal, point;

 void main()
 {
     vec3 c_d = vec3(gl_FrontMaterial.diffuse);
     vec3 c_a = vec3(gl_FrontMaterial.ambient);
     vec3 c_s = vec3(gl_FrontMaterial.specular);
     float p = gl_FrontMaterial.shininess;

     vec3 diffuse_sum = vec3(0.0, 0.0, 0.0);
     vec3 specular_sum = vec3(0.0, 0.0, 0.0);

     vec3 e_direction = normalize(-point);

     for (int i = 0; i < gl_MaxLights; i++) {
         vec3 l_p = vec3(gl_LightSource[i].position);

         // Attenuation (denominator)
         float d = length(point - l_p);
         float atten = 1.0 + gl_LightSource[i].quadraticAttenuation
                                    * pow(d, 2.0);

         vec3 l_c = vec3(gl_LightSource[i].ambient) / atten;
         vec3 l_direction = normalize(l_p - point);

         vec3 l_diffuse = l_c * max(0.0, dot(normal, l_direction));
         diffuse_sum += l_diffuse;

         vec3 l_specular = l_c * pow(max(0.0, dot(normal,
                                    normalize(e_direction + l_direction))), p);
         specular_sum += l_specular;
     }

     vec3 c = min(vec3(1, 1, 1),
                    c_a + diffuse_sum * c_d + specular_sum * c_s);

     gl_FragColor = vec4(c, 1.0);
 }
