using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public struct NavAgentComponent : IComponentData
{
    public Entity targetEntity;
    public bool pathCalculated;
    public int currentWaypoint;
    public float moveSpeed;
    public float nextPathCalculationTime;
}

public struct WaypointBuffer : IBufferElementData
{
    public float3 waypoint;
}