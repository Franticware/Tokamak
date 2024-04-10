#ifndef TOK_SAMPLE_GLAPP_H
#define TOK_SAMPLE_GLAPP_H

#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <tokamak.h>
#include <vector>

#define TOKAMAK_SAMPLE_TITLE_COMMON "Tokamak sample: "
extern const char* tokamakSampleTitle;

#define NUM_LIGHT 2
#define GROUND_Y 3.0f
#define CAMERA_SIZE 0.2f

extern neV4 vLightWorld[NUM_LIGHT];
extern neV4 vLightColor[NUM_LIGHT];

bool sdlGetAsyncKeyState(SDL_Keycode k);
bool sdlGetAsyncKeyStateOnce(SDL_Keycode k);

void MyAppInit();
void OnMyAppFrameMove(double fTime, float fElapsedTime);
void OnMyAppFrameRender();
void OnMyAppDestroyDevice();
void MyAppKeyboardProc(SDL_Keycode k, bool bKeyDown, bool bAltDown);

void MyRenderText(const char** str, int count);

inline void toGlMatrix(float m[16], const neT3& t)
{
    const float matrix[16] =
    {
        t.rot[0].X(),t.rot[0].Y(),t.rot[0].Z(),0,
        t.rot[1].X(),t.rot[1].Y(),t.rot[1].Z(),0,
        t.rot[2].X(),t.rot[2].Y(),t.rot[2].Z(),0,
        t.pos.X(),t.pos.Y(),t.pos.Z(),1,
    };
    memcpy(m, matrix, 16 * sizeof(float));
}

class CRenderPrimitive
{
public:
    float mColor[4];

    std::vector<float> mVerts;
    std::vector<float> mNormals;
    std::vector<uint16_t> mTris;

    CRenderPrimitive();
    void SetDiffuseColor(float r, float g, float b, float a);
    void SetGraphicBox(float Width, float Height, float Depth);
    void SetGraphicSphere(float radius);
    void SetGraphicCylinder(float radius, float length);
    void SetGraphicMesh(const char* strFilename);
    void SetGraphicMesh(const float* vert, int vertCount, const float* norm, int normCount, const uint16_t* ind, int indCount, int prim);
    void Render(const neT3 * matrix);
};

class SampleCamera
{
public:
    SampleCamera()
    {
        t.SetIdentity();
    }
    neV3 GetEyePt()
    {
        return mEyePt;
    }
    neV3 GetLookAtPt()
    {
        return mLookAtPt;
    }

    neV3 mEyePt;
    neV3 mLookAtPt;

    void SetViewParams(const neV3& vecEye, const neV3& vecAt)
    {
        mEyePt = vecEye;
        mLookAtPt = vecAt;

        t.pos.Set(0, 0, 0);
        t.rot.SetIdentity();

        neV3 z = vecEye - vecAt;
        z.Normalize();

        neV3 up;
        up.Set(0, 1, 0);

        neV3 x = up.Cross(z);
        x.Normalize();

        neV3 y = z.Cross(x);
        y.Normalize();

        t.rot[0] = x;
        t.rot[1] = y;
        t.rot[2] = z;

        t.rot.SetTranspose(t.rot);

        neT3 t2;
        t2.SetIdentity();
        t2.pos.Set(-vecEye);

        t = t * t2;
    }

    neT3 GetViewMatrix()
    {
        return t;
    }

    neT3 t;
};

extern SampleCamera g_Camera;

#endif // TOK_SAMPLE_GLAPP_H
