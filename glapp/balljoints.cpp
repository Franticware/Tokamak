#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "ball joints"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };

neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

const s32 MAX_OVERLAPPED_PAIR = 300;
const s32 WALL_NUMBER = 1;
const f32 EPSILON = 0.1f;

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

class CSampleBallJoints
{
public:
    CSampleBallJoints()
    {
        paused = false;
    }

    void Initialise();

    void Shutdown();

    void Process();

    void InititialisePhysics();

public:
    enum
    {
        N_BODY = 30,
    };

    neSimulator* sim;

    neRigidBody* box[N_BODY];

    CRenderPrimitive boxRenderPrimitives[N_BODY];

    neJoint* lastJoint;

    neAllocatorDefault all;

    nePerformanceReport perfReport;

    bool paused;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;
};

CSampleBallJoints sample;

void CSampleBallJoints::Initialise()
{
    InititialisePhysics();
}

void CSampleBallJoints::Process()
{
    s32 actOnBody = N_BODY - 1;

    if (sdlGetAsyncKeyState(SDLK_t))
    {
        neV3 vel;
        vel.Set(0.0f, 30.0f, 0.0f);
        box[actOnBody]->SetVelocity(vel);
    }
}

void CSampleBallJoints::Shutdown()
{
    neSimulator::DestroySimulator(sim);

    sim = NULL;
}

void CSampleBallJoints::InititialisePhysics()
{
    neV3 gravity;
    gravity.Set(0.0f, -8.0f, 0.0f);

    f32 linkLength = 1.2f;

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = N_BODY;
    sizeInfo.animatedBodiesCount = WALL_NUMBER;
    sizeInfo.geometriesCount = N_BODY + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = MAX_OVERLAPPED_PAIR;

    sizeInfo.controllersCount = 1;
    {
        // dont need any of these
        sizeInfo.rigidParticleCount = 0;
        sizeInfo.terrainNodesStartCount = 0;
    }

    sim = neSimulator::CreateSimulator(sizeInfo, &all, &gravity);

    neRigidBody* lastbox = NULL;

    for (s32 j = 0; j < N_BODY; j++)
    {
        f32 mass = 0.1f;

        neRigidBody* rigidBody = sim->CreateRigidBody();

        rigidBody->CollideConnected(true);

        neGeometry* geom = rigidBody->AddGeometry();

        neV3 boxSize;

        boxSize.Set(1.2f, 0.5f, 0.5f);

        geom->SetBoxSize(boxSize[0], boxSize[1], boxSize[2]);

        neV3 tensorSize;

        tensorSize = boxSize;
        ;

        rigidBody->UpdateBoundingInfo();

        // rigidBody->SetInertiaTensor(neBoxInertiaTensor(tensorSize, mass));

        rigidBody->SetInertiaTensor(neSphereInertiaTensor(tensorSize[0], mass));

        rigidBody->SetMass(mass);

        neV3 pos;

        if (j == 0)
        {
            pos.Set(-linkLength, 0.0f, 0.0f);
        }
        else if (j != 0)
        {
            pos.Set(-linkLength * (j + 1), 0.0f, 0.0f);
        }

        rigidBody->SetPos(pos);

        neJoint* joint = NULL;

        neT3 jointFrame;

        jointFrame.SetIdentity();

        if (j != 0)
        {
            joint = sim->CreateJoint(rigidBody, lastbox);
            // joint = sim->CreateJoint(lastbox, rigidBody);

            jointFrame.pos.Set(-linkLength * (0.5f + j), 0.0f, 0.0f);

            joint->SetJointFrameWorld(jointFrame);
        }
        if (j == N_BODY - 1)
        {
            lastJoint = joint;
        }

        if (joint)
        {
            joint->SetType(neJoint::NE_JOINT_BALLSOCKET);

            joint->Enable(true);
        }

        /* set up the graphical models */
        {
            this->boxRenderPrimitives[j].SetGraphicBox(boxSize[0], boxSize[1], boxSize[2]);
            geom->SetUserData(&(boxRenderPrimitives[j]));
            this->boxRenderPrimitives[j].SetDiffuseColor(0.8f, 0.2f, 0.2f, 1.0f);
        }
        box[j] = rigidBody;

        lastbox = rigidBody;
    }
    if (lastJoint)
    {
        lastJoint->SetEpsilon(EPSILON);

        lastJoint->SetIteration(5);
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
    vecEye.Set(-10.0f, 5.0f, 50.0f);
    neV3 vecAt;
    vecAt.Set(-30.0f, 0.0f, 0.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    for (s32 i = 0; i < NUM_LIGHT; i++)
    {
        vLightWorld[i].Normalize();
    }
    sample.Initialise();
}

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
    neT3 t;
    t = sample.ground->GetTransform();
    sample.groundRender.Render(&t);

    // Display the boxes
    for (s32 i = 0; i < sample.N_BODY; i++)
    {
        t = sample.box[i]->GetTransform();
        sample.boxRenderPrimitives[i].Render(&t);
    }

    const char* str[8];

    str[0] = "Tokamak demo - chain of 30 boxes connected by ball joints - (c) 2010 Tokamak Ltd";
    str[1] = "Controls:";
    str[2] = "'P' -> pause/unpause the simulation";
    str[3] = "'T' -> lift the end of the chain";
    str[4] = "This sample demonstrate the ball joint.";
    str[5] = "Note the box can contact each other and the ground is modelled";
    str[6] = "as a box, rather than just a plane.";
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

        default:
            break;
        }
    }
}
