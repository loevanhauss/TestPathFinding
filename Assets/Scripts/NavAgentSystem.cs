using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine.Experimental.AI;

public partial struct NavAgentSystem : ISystem
{
    private const int MaxPathSize = 100;

    public void OnUpdate(ref SystemState state)
    {
        double time = SystemAPI.Time.ElapsedTime;
        float deltaTime = SystemAPI.Time.DeltaTime;

        foreach (var (navAgent, transform, waypointBuffer) in
                 SystemAPI.Query<RefRW<NavAgentComponent>, RefRW<LocalTransform>, DynamicBuffer<WaypointBuffer>>())
        {
            if (navAgent.ValueRO.nextPathCalculationTime < time)
            {
                navAgent.ValueRW.nextPathCalculationTime = (float)time + 1f;
                navAgent.ValueRW.pathCalculated = false;
                CalculatePath(navAgent, transform, waypointBuffer, ref state);
            }
            else
            {
                Move(navAgent, transform, waypointBuffer, deltaTime);
            }
        }
    }

    private static void Move(
        RefRW<NavAgentComponent> navAgent,
        RefRW<LocalTransform> transform,
        DynamicBuffer<WaypointBuffer> waypointBuffer,
        float deltaTime)
    {
        if (waypointBuffer.Length == 0)
            return;

        int index = math.min(navAgent.ValueRO.currentWaypoint, waypointBuffer.Length - 1);
        float3 position = transform.ValueRO.Position;
        float3 target = waypointBuffer[index].waypoint;

        if (math.distance(position, target) < 0.4f)
        {
            if (index + 1 < waypointBuffer.Length)
            {
                index += 1;
                navAgent.ValueRW.currentWaypoint = index;
                target = waypointBuffer[index].waypoint;
            }
            else
            {
                return; // arrivé au bout du chemin
            }
        }

        float3 direction = target - position;
        float3 flatDirection = new float3(direction.x, 0, direction.z);

        if (math.lengthsq(flatDirection) > 1e-6f)
        {
            transform.ValueRW.Rotation = math.slerp(
                transform.ValueRO.Rotation,
                quaternion.LookRotationSafe(flatDirection, math.up()),
                math.saturate(deltaTime * 5f));
        }

        transform.ValueRW.Position += math.normalizesafe(direction) * navAgent.ValueRO.moveSpeed * deltaTime;
    }

    private static void CalculatePath(
        RefRW<NavAgentComponent> navAgent,
        RefRW<LocalTransform> transform,
        DynamicBuffer<WaypointBuffer> waypointBuffer,
        ref SystemState state)
    {
        Entity target = navAgent.ValueRO.targetEntity;
        if (target == Entity.Null || !state.EntityManager.HasComponent<LocalTransform>(target))
            return;

        float3 fromPosition = transform.ValueRO.Position;
        float3 toPosition = state.EntityManager.GetComponentData<LocalTransform>(target).Position;
        float3 extents = new float3(1, 1, 1);

        NavMeshQuery query = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.TempJob, 1000);

        NavMeshLocation fromLocation = query.MapLocation(fromPosition, extents, 0);
        NavMeshLocation toLocation = query.MapLocation(toPosition, extents, 0);

        if (query.IsValid(fromLocation) && query.IsValid(toLocation))
        {
            PathQueryStatus status = query.BeginFindPath(fromLocation, toLocation);

            if ((status & PathQueryStatus.InProgress) != 0)
            {
                status = query.UpdateFindPath(100, out int iterationsPerformed);
            }

            if ((status & PathQueryStatus.Success) != 0)
            {
                query.EndFindPath(out int pathSize);

                NativeArray<PolygonId> polygonIds = new NativeArray<PolygonId>(pathSize, Allocator.Temp);
                NativeArray<NavMeshLocation> straightPath = new NativeArray<NavMeshLocation>(MaxPathSize, Allocator.Temp);
                NativeArray<StraightPathFlags> straightPathFlags = new NativeArray<StraightPathFlags>(MaxPathSize, Allocator.Temp);
                NativeArray<float> vertexSide = new NativeArray<float>(MaxPathSize, Allocator.Temp);
                int straightPathCount = 0;

                query.GetPathResult(polygonIds);

                PathQueryStatus returningStatus = PathUtils.FindStraightPath(
                    query,
                    fromPosition,
                    toPosition,
                    polygonIds,
                    pathSize,
                    ref straightPath,
                    ref straightPathFlags,
                    ref vertexSide,
                    ref straightPathCount,
                    MaxPathSize);

                if ((returningStatus & PathQueryStatus.Success) != 0)
                {
                    waypointBuffer.Clear();

                    for (int i = 0; i < straightPathCount; i++)
                    {
                        waypointBuffer.Add(new WaypointBuffer { waypoint = straightPath[i].position });
                    }

                    navAgent.ValueRW.currentWaypoint = 0;
                    navAgent.ValueRW.pathCalculated = true;
                }

                polygonIds.Dispose();
                straightPath.Dispose();
                straightPathFlags.Dispose();
                vertexSide.Dispose();
            }
        }

        query.Dispose();
    }
}