using System;
using MessagePack;
using UnityEngine;

[MessagePackObject]
[Serializable]
public class UdpMsg
{
    [Key(0)]
    public byte cmd;
    [Key(1)]
    public float[] data;

    public UdpMsg() { }

    public UdpMsg(byte cmd, float[] data)
    {
        this.cmd = cmd;
        this.data = data;
    }
}
