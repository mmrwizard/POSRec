#version 330

//////////////////////////////////////////////////////////////////////////////////////
//
//
//	Bilateral filtering
//
//		C.B.    - 08/16/2008
//      D.G-M.  - 10/20/2010
//      D.G-M.  - 02/18/2014
//
//	IN:
//		s2_I	-	Image to blur
//		s2_D	-	Modulating depth image
//
//	OUT:
//		Filtered image
//
///////////////////////////////////////////////////////////////////////////////////////
//#extension GL_ARB_draw_buffers : enable

/****************************************************/
uniform	sampler2D	s2_I;
uniform	sampler2D	s2_D;
uniform	float		SX;
uniform	float		SY;
uniform	int		    NHalf;					//	half filter size (<= 7!)
uniform float		DistCoefs[64];			//	pixel distance based damping coefs (max = 8*8).
uniform float		SigmaDepth;				//  pixel depth distribution variance
in vec2 uv;

out vec4 fragolor;
/****************************************************/

void main (void)
{
	float	z		= texture(s2_D,uv).r;

	float	wsum	=	0.0;				// sum of all weights
	vec3	csum	=	vec3(0.0);			// sum of all contributions
	vec2	coordi	=	vec2(0.0,0.0);		// ith neighbor position

	for(int c=-NHalf; c<=NHalf; c++)
	{
		//neighbor position (X)
        coordi.x = float(c)/SX;
		int cabs;
		if (c < 0)
			cabs = -c;
		else
			cabs = c;

        for(int d=-NHalf; d<=NHalf; d++)
        {
            //neighbor position (Y)
            coordi.y    = float(d)/SY;

            //neighbor color
            vec3 ci	=	texture(s2_I,uv+coordi).rgb;

            //pixel distance based damping
			int dabs;
			if (d < 0)
				dabs = -d;
			else
				dabs = d;
            float fi	=	DistCoefs[cabs*(NHalf+1)+dabs];

            //pixel depth difference based damping
            if (SigmaDepth > 0.0)
            {
				//neighbor depth
				float zi	=	texture(s2_D,uv+coordi).r;
				float dz	=	(z-zi)/SigmaDepth;
				fi			*=	exp(-dz*dz/2.0);
			}

            csum	+=	ci * fi;
            wsum	+=	fi;
        }
	}

	//output
	fragolor = vec4(csum/wsum,1.0);
}

