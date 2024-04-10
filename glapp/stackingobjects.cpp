#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "stacking objects"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

#define PYRAMID
//#define COLLISION_CALLBACK_ON

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };
neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

// const int32_t MAX_OVERLAPPED_PAIR = 300;
const int32_t WALL_NUMBER = 1;
// const float EPSILON  = 0.1f;

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

class CSampleStackingObjects
{
public:
    CSampleStackingObjects()
    {
        paused = false;
    }

    void Initialise();

    void Shutdown();

    void Process();

    void InititialisePhysics();

    void MakeStack(neV3 position, int32_t& index);

    void MakeBullet(int);

    void Reset();

public:
    enum
    {
        STACK_HEIGHT = 10, // 4,

        N_STACKS = 5,

        N_DEPTH = 5,

        N_BULLET = 5,

#ifdef PYRAMID
        N_BODY = 55 + N_BULLET,
#else
        N_BODY = STACK_HEIGHT * N_STACKS * N_DEPTH + N_BULLET,
#endif
    };

    neSimulator* sim;

    neRigidBody* box[N_BODY];

    CRenderPrimitive boxRenderPrimitives[N_BODY];

    neAllocatorDefault all;

    nePerformanceReport perfReport;

    bool paused;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;
};

CSampleStackingObjects sample;

void CSampleStackingObjects::Initialise()
{
    InititialisePhysics();
}

void CSampleStackingObjects::Process()
{
    if (sdlGetAsyncKeyState(SDLK_r))
    {
        Reset();
        return;
    }

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

        pos = pos + dir * 3.0f;

        box[nextBullet]->SetPos(pos);
        box[nextBullet]->SetVelocity(dir * 40.0f);

        nextBullet = (nextBullet + 1) % N_BULLET;
    }
}

void CSampleStackingObjects::Shutdown()
{
    neSimulator::DestroySimulator(sim);

    sim = NULL;
}

void CSampleStackingObjects::Reset()
{
    Shutdown();

    InititialisePhysics();
}

void CSampleStackingObjects::MakeStack(neV3 position, int32_t& index)
{
    (void)position;
    const float groundLevel = -10.0f;

    int32_t cur = 0;
    ;

#ifndef PYRAMID

    for (int32_t i = 0; i < STACK_HEIGHT; i++)
    {
        cur = index + i;

        box[cur] = sim->CreateRigidBody();

        neGeometry* geom = box[cur]->AddGeometry();

        neV3 boxSize1;
        boxSize1.Set(0.8f, 0.8f, 0.8f);
        // neV3 boxSize1; boxSize1.Set(0.3, 1.9, 0.3);

        geom->SetBoxSize(boxSize1[0], boxSize1[1], boxSize1[2]);

        box[cur]->UpdateBoundingInfo();

        float mass = 1.0f;

        box[cur]->SetInertiaTensor(neBoxInertiaTensor(boxSize1[0], boxSize1[1], boxSize1[2], mass));

        box[cur]->SetMass(mass);

        neV3 pos;

        pos.Set(position[0], groundLevel + boxSize1[1] * (i + 0.5f) + 0.0f, position[2]);

        box[cur]->SetPos(pos);

        boxRenderPrimitives[cur].SetGraphicBox(boxSize1[0], boxSize1[1], boxSize1[2]);

        if ((cur % 4) == 0)
        {
            boxRenderPrimitives[cur].SetDiffuseColor(0.8f, 0.2f, 0.2f, 1.0f);
        }
        else if ((cur % 4) == 1)
        {
            boxRenderPrimitives[cur].SetDiffuseColor(0.2f, 0.1f, 0.8f, 1.0f);
        }
        else if ((cur % 4) == 2)
        {
            boxRenderPrimitives[cur].SetDiffuseColor(0.0f, 0.8f, 0.0f, 1.0f);
        }
        else
        {
            boxRenderPrimitives[cur].SetDiffuseColor(0.8f, 0.8f, 0.2f, 1.0f);
        }

        box[cur]->SetUserData(&boxRenderPrimitives[cur]);
    }
    index = cur + 1;

#else

    neV3 boxRadii;
    boxRadii.Set(.5f, .5f, .5f);
    neV3 boxSize;
    boxSize = boxRadii * 2.f;
    float stepX = boxSize[0] + 0.05f;
    int32_t baseSize = 10;

    neV3 startPos;
    startPos.Set(0.0f, groundLevel + boxSize[1] * 0.5f, 0.0f);

    int k = 0;

    for (int i = 0; i < baseSize; i++)
    {
        int rowSize = baseSize - i;

        neV3 start;
        start.Set(-rowSize * stepX * 0.5f + stepX * 0.5f, i * boxSize[1], 0);

        for (int j = 0; j < rowSize; j++)
        {
            neV3 boxPos;
            boxPos.Set(start);

            neV3 shift;
            shift.Set(j * stepX, 0.0f, 0.0f);

            boxPos += shift;
            boxPos += startPos;

            cur = index + k;

            box[cur] = sim->CreateRigidBody();

            neGeometry* geom = box[cur]->AddGeometry();

            geom->SetBoxSize(boxSize[0], boxSize[1], boxSize[2]);

            box[cur]->UpdateBoundingInfo();

            float mass = 1.f;

            box[cur]->SetInertiaTensor(neBoxInertiaTensor(boxSize[0], boxSize[1], boxSize[2], mass));

            box[cur]->SetMass(mass);

            box[cur]->SetPos(boxPos);

            boxRenderPrimitives[cur].SetGraphicBox(boxSize[0], boxSize[1], boxSize[2]);

            if ((cur % 4) == 0)
            {
                boxRenderPrimitives[cur].SetDiffuseColor(0.8f, 0.2f, 0.2f, 1.0f);
            }
            else if ((cur % 4) == 1)
            {
                boxRenderPrimitives[cur].SetDiffuseColor(0.2f, 0.1f, 0.8f, 1.0f);
            }
            else if ((cur % 4) == 2)
            {
                boxRenderPrimitives[cur].SetDiffuseColor(0.0f, 0.8f, 0.0f, 1.0f);
            }
            else
            {
                boxRenderPrimitives[cur].SetDiffuseColor(0.8f, 0.8f, 0.2f, 1.0f);
            }

            box[cur]->SetUserData(&boxRenderPrimitives[cur]);

            k++;
        }
    }
    index = cur + 1;
#endif
}

void CSampleStackingObjects::MakeBullet(int32_t index)
{

    neV3 boxSize;
    boxSize.Set(1.0f, 1.0f, 1.0f);

    int32_t cur = N_BODY - 1 - index;

    box[cur] = sim->CreateRigidBody();

    neGeometry* geom = box[cur]->AddGeometry();

    geom->SetBoxSize(boxSize);

    box[cur]->UpdateBoundingInfo();

    float mass = 0.6f;

    box[cur]->SetInertiaTensor(neBoxInertiaTensor(boxSize, mass));

    box[cur]->SetMass(mass);

    neV3 pos;

    pos.Set(-10.0f, -8.75f + index * 3.0f + 3, 0.0f);

    box[cur]->SetPos(pos);

    // graphic
    boxRenderPrimitives[cur].SetGraphicBox(boxSize[0], boxSize[1], boxSize[2]);
    boxRenderPrimitives[cur].SetDiffuseColor(0.8f, 0.8f, 0.8f, 1.0f);

    box[cur]->SetUserData(&boxRenderPrimitives[cur]);
}

void CollisionCallback(neCollisionInfo& collisionInfo)
{
    if (collisionInfo.typeA == NE_RIGID_BODY)
    {
        neRigidBody* rbA = (neRigidBody*)collisionInfo.bodyA;

        void* data = rbA->GetUserData().p;

        if (data == 0)
        {
            return;
        }

        CRenderPrimitive* render = reinterpret_cast<CRenderPrimitive*>(data);
        render->SetDiffuseColor(1.0, 0, 0, 1.0f);
    }
    if (collisionInfo.typeB == NE_RIGID_BODY)
    {
        neRigidBody* rbB = (neRigidBody*)collisionInfo.bodyB;

        void* data = rbB->GetUserData().p;

        if (data == 0)
        {
            return;
        }

        CRenderPrimitive* render = reinterpret_cast<CRenderPrimitive*>(data);
        render->SetDiffuseColor(0, 0.5f, 0.5, 1.0f);
    }
}

void CSampleStackingObjects::InititialisePhysics()
{
    neV3 gravity;
    gravity.Set(0.0f, -9.0f, 0.0f);

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = N_BODY;
    sizeInfo.animatedBodiesCount = WALL_NUMBER;
    sizeInfo.geometriesCount = N_BODY + WALL_NUMBER;
    int32_t totalBody = N_BODY + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = totalBody * (totalBody - 1) / 2;
    {
        // dont need any of these
        sizeInfo.rigidParticleCount = 0;
        sizeInfo.constraintsCount = 0;
        sizeInfo.terrainNodesStartCount = 0;
    }

    sim = neSimulator::CreateSimulator(sizeInfo, &all, &gravity);

#ifdef COLLISION_CALLBACK_ON

    sim->GetCollisionTable()->Set(0, 0, neCollisionTable::RESPONSE_IMPULSE_CALLBACK);

    sim->SetCollisionCallback(CollisionCallback);

#endif

    neV3 position;

    position.SetZero();

    int32_t i = 0;

#ifdef PYRAMID

    MakeStack(position, i);

#else
    float gap = 2.f;

    for (int32_t j = 0; j < N_STACKS; j++)
    {
        // position.Set(3.05f * j, 0.0f, 0.0f);

        for (int32_t k = 0; k < N_DEPTH; k++)
        {
            position.Set(gap * j, 0.0f, gap * k);

            MakeStack(position, i);
        }
    }
#endif

    for (int32_t jj = 0; jj < N_BULLET; jj++)
    {
        MakeBullet(jj);
    }

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
    vecEye.Set(-10.0f, 5.0f, 40.0f);
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
    ////////////////////////////////////////////////////////
    sample.Process();

    if (!sample.paused)
    {
        sample.sim->Advance(1.0f / 60.0f, 1);
    }
}

void OnMyAppFrameRender()
{
    neT3 t;
    t = sample.ground->GetTransform();
    t.MakeD3DCompatibleMatrix();
    sample.groundRender.Render(&t);

    // Display the boxes

    for (int32_t i = 0; i < sample.N_BODY; i++)
    {
        t = sample.box[i]->GetTransform();
        t.MakeD3DCompatibleMatrix();
        sample.boxRenderPrimitives[i].Render(&t);
    }

    const char* str[7];

    str[0] = "Tokamak demo - stacking of objects - (c) 2010 Tokamak Ltd";
    str[1] = "Controls:";
    str[2] = "'P' -> pause/unpause the simulation";
    str[3] = "'T' -> Fire block";
    str[4] = "'R' -> Reset";
    str[5] = "Demonstration of fast and stable stacking.";
    str[6] = "";

    MyRenderText(str, 7);
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
