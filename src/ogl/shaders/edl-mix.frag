#version 330

//////////////////////////////////////////////////////////////////////////////////////
//
//
//	EyeDome Lighting	-	Compositing
//
//		C.B.    - 04/23/2008
//		D.G-M.  - 10/21/2010
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
//#extension GL_ARB_draw_buffers : enable

/**************************************************/
uniform	sampler2D	s2_I1;	//	X1 scale
uniform sampler2D	s2_I2;	//	X2 scale
uniform sampler2D	s2_I4;	//	X4 scale
uniform sampler2D	s2_D;	// initial depth texture
//
uniform float		A0;
uniform float		A1;
uniform float		A2;
uniform int		absorb;
/**************************************************/

in vec2 uv;
out vec4 fragcolor;

void main (void)
{
	float d = texture(s2_D,uv).r;

	if( d > 0.99)
	{
		fragcolor.rgb = texture(s2_I1,uv).rgb;
		fragcolor.a = 1.;
		return;
	}

    //color version
	vec3 C;
	if(d>0.99 && absorb==1)
	{
		C = texture(s2_I1,uv).rgb;
	}
	else
	{
		vec3 C1 = texture(s2_I1,uv).rgb;
		vec3 C2 = texture(s2_I2,uv).rgb;
		vec3 C4 = texture(s2_I4,uv).rgb;
		C = (A0*C1 + A1*C2 + A2*C4) /(A0+A1+A2);
	}

	fragcolor = vec4(C.x,C.y,C.z,1.);
}
