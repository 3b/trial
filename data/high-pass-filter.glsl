#section FRAGMENT_SHADER
in vec2 uv;
out vec4 color;
uniform sampler2D previous_pass;
uniform float threshold = 1.0;

float color_luminance(vec3 color){
  return dot(color, vec3(0.299, 0.587, 0.114));
}

void main(){
  color = texture(previous_pass, uv);
  float sum = color_luminance(color);
  if(sum >= threshold){
    color = vec4(0);
  }
}
