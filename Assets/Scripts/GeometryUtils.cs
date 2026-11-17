using Unity.Mathematics;

public static class GeometryUtils
{
    // Points les plus proches entre le segment p0-p1 et le segment q0-q1
    public static float SegmentSegmentCPA(out float3 c0, out float3 c1,
        float3 p0, float3 p1, float3 q0, float3 q1)
    {
        float3 d1 = p1 - p0;
        float3 d2 = q1 - q0;
        float3 r = p0 - q0;
        float a = math.dot(d1, d1);
        float e = math.dot(d2, d2);
        float f = math.dot(d2, r);
        const float eps = 1e-6f;
        float s, t;

        if (a <= eps && e <= eps)
        {
            s = t = 0f;
        }
        else if (a <= eps)
        {
            s = 0f;
            t = math.clamp(f / e, 0f, 1f);
        }
        else
        {
            float c = math.dot(d1, r);
            if (e <= eps)
            {
                t = 0f;
                s = math.clamp(-c / a, 0f, 1f);
            }
            else
            {
                float b = math.dot(d1, d2);
                float denom = a * e - b * b;
                s = denom > eps ? math.clamp((b * f - c * e) / denom, 0f, 1f) : 0f;
                t = (b * s + f) / e;
                if (t < 0f)
                {
                    t = 0f;
                    s = math.clamp(-c / a, 0f, 1f);
                }
                else if (t > 1f)
                {
                    t = 1f;
                    s = math.clamp((b - c) / a, 0f, 1f);
                }
            }
        }

        c0 = p0 + d1 * s;
        c1 = q0 + d2 * t;
        return math.distancesq(c0, c1);
    }
}