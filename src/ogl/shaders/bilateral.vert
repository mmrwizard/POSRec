#version 330

//////////////////////////////////////////////////////////////////////////////////////
//
//
//	Bilateral filtering
//
//		C.B.    - 08/16/2008
//      D.G-M.  - 10/20/2010
//
//	IN:
//		s2_I	-	Image to blur
//		s2_D	-	Modulating depth image
//
//	OUT:
//		Filtered image
//
///////////////////////////////////////////////////////////////////////////////////////
//#extension GL_EXT_gpu_shader4 : enable

uniform mat4 mvp;
in vec3 position;
in vec2 texcoord;
out vec2 uv;

void	main ()
{
	uv = texcoord;
	gl_Position = mvp * vec4(position, 1.0);
}
