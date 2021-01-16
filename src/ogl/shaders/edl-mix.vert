#version 330

//////////////////////////////////////////////////////////////////////////////////////
//
//
//	EyeDome Lighting	-	Compositing
//
//		C.B. - 23 avril 2008
//
//	IN:	
//		s2_I1		-	full scale shading image
//		s2_I2		-	half-size shading image
//		s2_I4		-	quarter-size shading image
//		s2_D		-	depth image
//	OUT:	
//		composited image
//
//////////////////////////////////////////////////////////////////////////////////////
uniform mat4 mvp;
in vec3 position;
in vec2 texcoord;
out vec2 uv;
void	main ()
{
	uv = texcoord;
	gl_Position = mvp * vec4(position, 1.0);
}
