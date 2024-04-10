#include "landscape2_x_data.h"
#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "rigid particles and terrain"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };

neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

// const int32_t MAX_OVERLAPPED_PAIR = 300;
const int32_t WALL_NUMBER = 1;
// const float EPSILON = 0.1f;

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

class CSampleRigidParticlesAndTerrain
{
public:
    CSampleRigidParticlesAndTerrain()
    {
        paused = false;
    }

    void Initialise();

    void Shutdown();

    void Process();

    void InititialisePhysics();

    void MakeParticle(neV3 position, int32_t index);

public:
    enum
    {
        NUMBER_OF_PARTICLES = 100,

        GEOMETRY_PER_BODY = 1,
    };

    neSimulator* sim;

    neRigidBody* box[NUMBER_OF_PARTICLES];

    CRenderPrimitive boxRenderPrimitives[NUMBER_OF_PARTICLES * GEOMETRY_PER_BODY];

    neAllocatorDefault all;

    nePerformanceReport perfReport;

    bool paused;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;

    CRenderPrimitive terrainRender;

    int32_t nextBullet;
};

CSampleRigidParticlesAndTerrain sample;

void CSampleRigidParticlesAndTerrain::Initialise()
{
    InititialisePhysics();
    nextBullet = 0;
}

void CSampleRigidParticlesAndTerrain::Process()
{
    static int32_t nextBullet = 0;
    if (sdlGetAsyncKeyStateOnce(SDLK_t))
    {
        neV3 pos;
        neV3 dir;
        neV3 eyept = g_Camera.GetEyePt();
        neV3 lookatpt = g_Camera.GetLookAtPt();
        neV3 d3ddir;
        d3ddir = lookatpt - eyept;

        pos.Set(eyept);
        dir.Set(d3ddir);
        dir.Normalize();

        pos = pos + dir * 10.0f;

        box[nextBullet]->SetPos(pos);
        box[nextBullet]->SetVelocity(dir * 20.0f);

        nextBullet = (nextBullet + 1) % NUMBER_OF_PARTICLES;
    }
}

void CSampleRigidParticlesAndTerrain::Shutdown()
{
    sim->FreeTerrainMesh();

    neSimulator::DestroySimulator(sim);

    sim = NULL;
}

void CSampleRigidParticlesAndTerrain::MakeParticle(neV3 position, int32_t index)
{
    // const float groundLevel = -10.0f;

    int32_t cur;

    cur = index;

    box[cur] = sim->CreateRigidParticle();

    neGeometry* geom = box[cur]->AddGeometry();

    neV3 boxSize1;
    boxSize1.Set(1.f, 1.f, 1.f);

    geom->SetBoxSize(boxSize1[0], boxSize1[1], boxSize1[2]);

    box[cur]->UpdateBoundingInfo();

    float mass = 0.1f;

    box[cur]->SetInertiaTensor(neBoxInertiaTensor(boxSize1[0], boxSize1[1], boxSize1[2], mass));

    box[cur]->SetMass(mass);

    box[cur]->SetPos(position);

    // graphic
    boxRenderPrimitives[cur].SetGraphicBox(boxSize1[0], boxSize1[1], boxSize1[2]);

    if ((cur % 4) == 0)
    {
        boxRenderPrimitives[cur].SetDiffuseColor(0.8f, 0.2f, 0.2f, 1.0f);

    }
    else if ((cur % 4) == 1)
    {
        boxRenderPrimitives[cur].SetDiffuseColor(0.2f, 0.4f, 0.6f, 1.0f);

    }
    else if ((cur % 4) == 2)
    {
        boxRenderPrimitives[cur].SetDiffuseColor(0.2f, 0.6f, 0.2f, 1.0f);
    }
    else
    {
        boxRenderPrimitives[cur].SetDiffuseColor(0.6f, 0.6f, 0.2f, 1.0f);
    }
}

void CSampleRigidParticlesAndTerrain::InititialisePhysics()
{
    neV3 gravity;
    gravity.Set(0.0f, -8.0f, 0.0f);

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidParticleCount = NUMBER_OF_PARTICLES;
    sizeInfo.animatedBodiesCount = WALL_NUMBER;
    sizeInfo.geometriesCount = NUMBER_OF_PARTICLES * GEOMETRY_PER_BODY + WALL_NUMBER;

    {
        // dont need any of these
        sizeInfo.rigidBodiesCount = 0;
        sizeInfo.constraintsCount = 0;
    }

    int32_t totalBody = NUMBER_OF_PARTICLES + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = totalBody * (totalBody - 1) / 2;

    sim = neSimulator::CreateSimulator(sizeInfo, &all, &gravity);

    neV3 position;

    position.SetZero();

    for (int32_t j = 0; j < NUMBER_OF_PARTICLES; j++)
    {
        position.Set(0.0f, 2.0f * j + 20.0f, 0.0f);
        // position.Set(13.5f, 20.0f, 1.5f);

        MakeParticle(position, j);
    }

    // SetUpTerrain

    terrainRender.SetGraphicMesh(terVert, sizeof(terVert) / sizeof(float) / 3, terNorm, sizeof(terNorm) / sizeof(float) / 3, terQuad, sizeof(terQuad) / sizeof(uint16_t), 4);
    terrainRender.SetDiffuseColor(0.1f, 0.5f, 0.1f, 1.0f);

    neTriangleMesh triMesh;

    triMesh.vertexCount = terrainRender.mVerts.size() / 3;

    triMesh.triangleCount = terrainRender.mTris.size() / 3;

    neV3* verts = new neV3[triMesh.vertexCount];

    //
    for (int i = 0; i != triMesh.vertexCount; ++i)
    {
        verts[i].Set(terrainRender.mVerts[i * 3], terrainRender.mVerts[i * 3 + 1], terrainRender.mVerts[i * 3 + 2]);
    }

    triMesh.vertices = verts;

    neTriangle* tri = new neTriangle[triMesh.triangleCount];

    int32_t* triindex = new int32_t[triMesh.triangleCount * 3];

    for (int i = 0; i != triMesh.triangleCount * 3; ++i)
    {
        triindex[i] = terrainRender.mTris[i];
    }

    //

    for (int32_t i = 0; i < triMesh.triangleCount; i++)
    {
        tri[i].indices[0] = triindex[i * 3];
        tri[i].indices[1] = triindex[i * 3 + 1];
        tri[i].indices[2] = triindex[i * 3 + 2];
        tri[i].materialID = 0;
        tri[i].flag = neTriangle::NE_TRI_TRIANGLE;
        // tri[i].flag = neTriangle::NE_TRI_HEIGHT_MAP;
    }
    triMesh.triangles = tri;

    sim->SetTerrainMesh(&triMesh);

    // SetUpRoom

    ground = sim->CreateAnimatedBody();

    neGeometry* geom = ground->AddGeometry();

    geom->SetBoxSize(gFloor.boxSize);

    ground->UpdateBoundingInfo();

    ground->SetPos(gFloor.pos);

    groundRender.SetGraphicBox(gFloor.boxSize[0], gFloor.boxSize[1], gFloor.boxSize[2]);
}

void MyAppInit()
{
    neV3 vecEye;
    vecEye.Set(-10.0f, 25.0f, 40.0f);
    neV3 vecAt;
    vecAt.Set(0.0f, 0.0f, 1.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    for (int32_t i = 0; i < NUM_LIGHT; i++)
    {
        vLightWorld[i].Normalize();
    }

    sample.Initialise();
};

void OnMyAppFrameMove(double fTime, float fElapsedTime)
{
    (void)fTime;
    (void)fElapsedTime;
    sample.Process();

    if (!sample.paused)
    {
        sample.sim->Advance(1.0f / 60.0f, 1);
    }
}

void OnMyAppFrameRender()
{
    neT3 terrainTrans;
    terrainTrans.SetIdentity();
    sample.terrainRender.Render(&terrainTrans);

    neT3 t;
    t = sample.ground->GetTransform();
    sample.groundRender.Render(&t);

    // Display the boxes

    for (int32_t i = 0; i < sample.NUMBER_OF_PARTICLES; i++)
    {
        t = sample.box[i]->GetTransform();
        t.MakeD3DCompatibleMatrix();
        sample.boxRenderPrimitives[i].Render(&t);
    }

    const char* str[8];

    str[0] = "Tokamak demo - Rigid Particles and Terrain Demo - (c) 2010 Tokamak Ltd";
    str[1] = "Controls:";
    str[2] = "'P' -> pause/unpause the simulation";
    str[3] = "'T' -> Fire particles";
    str[4] = "100 rigid particles interact with arbitrary landscape mesh.";
    str[5] = "Rigid particles are light-weight rigid bodies which doesn't";
    str[6] = "collide with each other, designed for particle effects";
    str[7] = "";

    MyRenderText(str, 8);
}

void OnMyAppDestroyDevice()
{
    sample.Shutdown();
}

void MyAppKeyboardProc(SDL_Keycode nChar, bool bKeyDown, bool bAltDown)
{
    (void)bAltDown;
    if (bKeyDown)
    {
        switch (nChar)
        {
        case SDLK_r:
        {
        } break;

        case SDLK_p:
        {
            sample.paused = !sample.paused;
        }
        break;

        default:
            break;
        }
    }
}
