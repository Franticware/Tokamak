#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "breakage objects"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };

neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

const int32_t MAX_OVERLAPPED_PAIR = 300;
const int32_t WALL_NUMBER = 1;
// const f32 EPSILON  = 0.1f;

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

void BreakageCallbackFn(neByte* originalBody, neBodyType bodyType, neGeometry* brokenGeometry, neRigidBody* newBody);

class CSampleBreakageObjects
{
public:
    CSampleBreakageObjects()
    {
        paused = false;
    }

    void Initialise();

    void Shutdown();

    void Process();

    void InititialisePhysics();

    void MakeTable(neV3 position, int32_t index);

public:
    enum
    {
        GEOMETRY_PER_TABLE = 5,

        NUMBER_OF_TABLES = 15,

        NUMBER_OF_BODIES = NUMBER_OF_TABLES * GEOMETRY_PER_TABLE,
    };

    neSimulator* sim;

    neRigidBody* rigidBodies[NUMBER_OF_BODIES];

    CRenderPrimitive boxRenderPrimitives[NUMBER_OF_BODIES];

    neAllocatorDefault all;

    nePerformanceReport perfReport;

    bool paused;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;

    int32_t currentBodyCount;
};

CSampleBreakageObjects sample;

void CSampleBreakageObjects::Initialise()
{
    InititialisePhysics();

    currentBodyCount = NUMBER_OF_TABLES;
}

void CSampleBreakageObjects::Process()
{

    neV3 force;

    neBool dontSet = false;

    f32 v = 5.0f;

    if (sdlGetAsyncKeyState(SDLK_t))
    {
        force.Set(0.0f, v, 0.0f);
    }
    else
    {
        dontSet = true;
    }

    if (!dontSet)
    {
        for (int32_t i = 0; i < NUMBER_OF_TABLES; i++)
        {
            rigidBodies[i]->SetVelocity(force);
        }
    }
}

void CSampleBreakageObjects::Shutdown()
{
    neSimulator::DestroySimulator(sim);

    sim = NULL;
}

void BreakageCallbackFn(neByte* originalBody, neBodyType bodyType, neGeometry* brokenGeometry, neRigidBody* newBody)
{
    (void)originalBody;
    (void)bodyType;
    (void)brokenGeometry;

    sample.rigidBodies[sample.currentBodyCount] = (neRigidBody*)newBody;

    sample.currentBodyCount++;
}

void CSampleBreakageObjects::InititialisePhysics()
{
    neV3 gravity;
    gravity.Set(0.0f, -9.0f, 0.0f);

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = NUMBER_OF_BODIES;
    sizeInfo.animatedBodiesCount = WALL_NUMBER;
    sizeInfo.geometriesCount = NUMBER_OF_BODIES + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = MAX_OVERLAPPED_PAIR; // totalBody * (totalBody - 1) / 2;
    {
        // dont need any of these
        sizeInfo.rigidParticleCount = 0;
        sizeInfo.constraintsCount = 0;
        sizeInfo.terrainNodesStartCount = 0;
    }

    sim = neSimulator::CreateSimulator(sizeInfo, &all, &gravity);

    sim->SetBreakageCallback(BreakageCallbackFn);

    for (int32_t i = 0; i < NUMBER_OF_BODIES; i++)
    {
        rigidBodies[i] = NULL;
    }

    neV3 position;

    position.SetZero();

    for (int32_t j = 0; j < NUMBER_OF_TABLES; j++)
    {
        position.Set(20.0f + 4.0f * j, 30.0f + 5.0f * j, 0.0f);

        MakeTable(position, j);
    }

    // SetUpRoom

    ground = sim->CreateAnimatedBody();

    neGeometry* geom = ground->AddGeometry();

    geom->SetBoxSize(gFloor.boxSize);

    ground->UpdateBoundingInfo();

    ground->SetPos(gFloor.pos);

    groundRender.SetGraphicBox(gFloor.boxSize[0], gFloor.boxSize[1], gFloor.boxSize[2]);
}

struct TableData
{
    neV3 size;
    neV3 pos;
    neV3 colour;
    neGeometry::neBreakFlag breakFlag;
    f32 breakageMass;
    f32 breakageMagnitude;
    f32 breakageAbsorption;
};

TableData tableData[CSampleBreakageObjects::GEOMETRY_PER_TABLE] =
{
    { { 3.0f, 0.3f, 3.0f }, { 0.0f, 0.0f, 0.0f }, { 0.8f, 0.1f, 0.1f }, neGeometry::NE_BREAK_DISABLE, 2.0f, 0.0f, 0.0f },
    { { 0.4f, 1.0f, 0.4f }, { 1.3f, -0.65f, 1.3f }, { 0.1f, 0.1f, 0.8f }, neGeometry::NE_BREAK_NORMAL, 0.4f, 5.0f, 0.3f },
    { { 0.4f, 1.0f, 0.4f }, { -1.3f, -0.65f, 1.3f }, { 0.1f, 0.1f, 0.8f }, neGeometry::NE_BREAK_NORMAL, 0.4f, 5.0f, 0.3f },
    { { 0.4f, 1.0f, 0.4f }, { 1.3f, -0.65f, -1.3f }, { 0.1f, 0.1f, 0.8f }, neGeometry::NE_BREAK_NORMAL, 0.4f, 5.0f, 0.3f },
    { { 0.4f, 1.0f, 0.4f }, { -1.3f, -0.65f, -1.3f }, { 0.1f, 0.1f, 0.8f }, neGeometry::NE_BREAK_NORMAL, 0.4f, 5.0f, 0.3f },

};

void CSampleBreakageObjects::MakeTable(neV3 position, int32_t index)
{
    // const f32 groundLevel = -10.0f;

    neV3 tableSize;

    tableSize.Set(3.0f, 1.0f, 3.0f);

    int32_t cur;

    cur = index;

    rigidBodies[cur] = sim->CreateRigidBody();

    rigidBodies[cur]->SetInertiaTensor(neBoxInertiaTensor(tableSize, 2.0f));

    rigidBodies[cur]->SetMass(2.0f);

    // position[1] = groundLevel + 0.95f * (1 + cur) + (0.15f * cur);

    rigidBodies[cur]->SetPos(position);

    neQ rot;

    rot.X = ((f32)rand()) / (f32)RAND_MAX;
    rot.Y = ((f32)rand()) / (f32)RAND_MAX;
    rot.Z = ((f32)rand()) / (f32)RAND_MAX;
    rot.W = ((f32)rand()) / (f32)RAND_MAX;

    rot.Normalize();

    rigidBodies[cur]->SetRotation(rot);

    for (int32_t i = 0; i < GEOMETRY_PER_TABLE; i++)
    {
        neGeometry* geom = rigidBodies[cur]->AddGeometry();

        geom->SetUserData((uint32_t)(cur * GEOMETRY_PER_TABLE + i));

        geom->SetBoxSize(tableData[i].size);

        neT3 trans;

        trans.SetIdentity();

        trans.pos = tableData[i].pos;

        geom->SetTransform(trans);

        geom->SetBreakageFlag(tableData[i].breakFlag);

        geom->SetBreakageMass(tableData[i].breakageMass);

        geom->SetBreakageInertiaTensor(neBoxInertiaTensor(tableData[i].size, tableData[i].breakageMass));

        geom->SetBreakageMagnitude(tableData[i].breakageMagnitude);

        geom->SetBreakageAbsorption(tableData[i].breakageAbsorption);

        neV3 plane;

        plane.Set(0.0f, 1.0f, 0.0f);

        geom->SetBreakagePlane(plane);

        // graphic
        boxRenderPrimitives[geom->GetUserData().u].SetGraphicBox(tableData[i].size.X(), tableData[i].size.Y(), tableData[i].size.Z());

        if (i % 3 == 0)
        {
            boxRenderPrimitives[geom->GetUserData().u].SetDiffuseColor(0.8f, 0.2f, 0.2f, 1.0f);
        }
        else if (i % 3 == 1)
        {
            boxRenderPrimitives[geom->GetUserData().u].SetDiffuseColor(0.3f, 0.3f, 1.0f, 1.0f);
        }
        else
        {
            boxRenderPrimitives[geom->GetUserData().u].SetDiffuseColor(0.2f, 0.8f, 0.2f, 1.0f);
        }
    }
    rigidBodies[cur]->UpdateBoundingInfo();
}

void MyAppInit()
{
    neV3 vecEye;
    vecEye.Set(-10.0f, 5.0f, 45.0f);
    neV3 vecAt;
    vecAt.Set(47.0f, 0.0f, 1.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    for (int32_t i = 0; i < NUM_LIGHT; i++)
    {
        vLightWorld[i].Normalize();
    }

    sample.Initialise();
};

void OnMyAppFrameMove(double fTime, f32 fElapsedTime)
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
    t.MakeD3DCompatibleMatrix();
    sample.groundRender.Render(&t);

    // Display the boxes
    for (int32_t i = 0; i < sample.currentBodyCount; i++)
    {
        neT3 trans = sample.rigidBodies[i]->GetTransform();

        sample.rigidBodies[i]->BeginIterateGeometry();

        neGeometry* geom = sample.rigidBodies[i]->GetNextGeometry();

        while (geom)
        {
            uint32_t index = geom->GetUserData().u;

            neT3 geomTrans = geom->GetTransform();

            neT3 worldTrans = trans * geomTrans;

            worldTrans.MakeD3DCompatibleMatrix();

            sample.boxRenderPrimitives[index].Render(/*pd3dDevice,*/ &worldTrans);

            geom = sample.rigidBodies[i]->GetNextGeometry();
        }
    }

    // display text

    const char* str[9];

    str[0] = "Tokamak demo - build-in of breakage function - (c) 2010 Tokamak Ltd";
    str[1] = "Controls:";
    str[2] = "'P' -> pause/unpause the simulation";
    str[3] = "'T' -> Lift tables";
    str[4] = "Demonstrate the build in breaking function of Tokamak.";
    str[5] = "A rigid body is assemble with any number of geometries.";
    str[6] = "Each geometry can be told to break off at a certain";
    str[7] = "collision force. Tokamak will automatically spawn the";
    str[8] = "new fragment objects when the breaking occurs.";

    MyRenderText(str, 9);
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
