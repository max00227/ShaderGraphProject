using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

public class SmoothNormalPostprocessor : AssetPostprocessor
{
    // 凹陷度強度倍率（數值越大，淺溝槽的凹陷數值越明顯）
    private const float ConcavityIntensity = 2.0f;

    // 鄰居擴散次數（解決寬底環形溝槽中心平坦為 0 的問題，0 表示不擴散）
    private const int DiffusionSteps = 2;

    // 當模型匯入 Unity 時，會自動觸發這個函式
    void OnPostprocessModel(GameObject g)
    {
        // 為了避免影響到所有模型拖慢匯入速度，設定只有檔名包含 "_outline" 的 FBX 才會執行
        if (!assetPath.ToLower().Contains("_outline"))
            return;

        // 處理靜態模型 (MeshFilter)
        foreach (MeshFilter mf in g.GetComponentsInChildren<MeshFilter>())
        {
            if (mf.sharedMesh != null)
                ProcessMesh(mf.sharedMesh);
        }

        // 處理動態骨骼模型 (SkinnedMeshRenderer)
        foreach (SkinnedMeshRenderer smr in g.GetComponentsInChildren<SkinnedMeshRenderer>())
        {
            if (smr.sharedMesh != null)
                ProcessMesh(smr.sharedMesh);
        }
    }

    void ProcessMesh(Mesh mesh)
    {
        Vector3[] vertices = mesh.vertices;
        Vector3[] normals = mesh.normals;
        int[] triangles = mesh.triangles;
        int vertexCount = vertices.Length;

        if (vertexCount == 0)
            return;

        if (normals == null || normals.Length != vertexCount)
        {
            mesh.RecalculateNormals();
            normals = mesh.normals;
        }

        // ==========================================
        // 步驟 1：合併相同座標的頂點（處理硬邊與 UV 接縫分裂）並計算平滑法線
        // ==========================================
        Dictionary<Vector3, int> posToWeldedIndex = new Dictionary<Vector3, int>(vertexCount);
        int[] vertexToWelded = new int[vertexCount];
        List<Vector3> weldedPositions = new List<Vector3>();
        List<Vector3> weldedNormals = new List<Vector3>();

        for (int i = 0; i < vertexCount; i++)
        {
            Vector3 p = vertices[i];
            if (!posToWeldedIndex.TryGetValue(p, out int weldedIdx))
            {
                weldedIdx = weldedPositions.Count;
                posToWeldedIndex[p] = weldedIdx;
                weldedPositions.Add(p);
                weldedNormals.Add(normals[i]);
            }
            else
            {
                weldedNormals[weldedIdx] += normals[i]; // 累加分裂的硬邊法線
            }

            vertexToWelded[i] = weldedIdx;
        }

        int weldedCount = weldedPositions.Count;
        for (int i = 0; i < weldedCount; i++)
        {
            weldedNormals[i] = weldedNormals[i].normalized;
        }

        // 將平滑法線寫入 UV2 (索引值為 2，對應 Shader Graph 的 UV2)
        List<Vector3> smoothNormals = new List<Vector3>(vertexCount);
        for (int i = 0; i < vertexCount; i++)
        {
            smoothNormals.Add(weldedNormals[vertexToWelded[i]]);
        }
        mesh.SetUVs(2, smoothNormals);

        // ==========================================
        // 步驟 2：建立焊接頂點的鄰接表 (Adjacency List)
        // ==========================================
        HashSet<int>[] neighbors = new HashSet<int>[weldedCount];
        for (int i = 0; i < weldedCount; i++)
        {
            neighbors[i] = new HashSet<int>();
        }

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int w0 = vertexToWelded[triangles[i]];
            int w1 = vertexToWelded[triangles[i + 1]];
            int w2 = vertexToWelded[triangles[i + 2]];

            if (w0 != w1) { neighbors[w0].Add(w1); neighbors[w1].Add(w0); }
            if (w1 != w2) { neighbors[w1].Add(w2); neighbors[w2].Add(w1); }
            if (w2 != w0) { neighbors[w2].Add(w0); neighbors[w0].Add(w2); }
        }

        // ==========================================
        // 步驟 3：計算凹陷度（支援環形溝狀與一般凹陷）
        // ==========================================
        float[] maxConcavity = new float[weldedCount]; // 針對環形溝狀（主曲率最大凹陷，不受圓環外凸抵消）
        float[] avgConcavity = new float[weldedCount]; // 針對一般全方位凹坑（平均曲率）

        for (int i = 0; i < weldedCount; i++)
        {
            int count = neighbors[i].Count;
            if (count == 0) continue;

            Vector3 p = weldedPositions[i];
            Vector3 n = weldedNormals[i];
            float maxDot = 0f;
            float sumDot = 0f;

            foreach (int neighborIdx in neighbors[i])
            {
                Vector3 dir = (weldedPositions[neighborIdx] - p).normalized;
                // dir 與平滑法線內積：> 0 代表周圍頂點往法線方向翹起（凹陷），< 0 代表凸起
                float dot = Vector3.Dot(dir, n);

                if (dot > maxDot)
                {
                    maxDot = dot;
                }
                sumDot += dot;
            }

            maxConcavity[i] = maxDot;
            avgConcavity[i] = sumDot / count;
        }

        // ==========================================
        // 步驟 4：鄰居擴散（讓寬底環形溝槽的中心也能保留凹陷度）
        // ==========================================
        for (int step = 0; step < DiffusionSteps; step++)
        {
            float[] nextMax = new float[weldedCount];
            for (int i = 0; i < weldedCount; i++)
            {
                int count = neighbors[i].Count;
                if (count == 0)
                {
                    nextMax[i] = maxConcavity[i];
                    continue;
                }

                float sum = maxConcavity[i];
                foreach (int neighborIdx in neighbors[i])
                {
                    sum += maxConcavity[neighborIdx];
                }
                // 保留自身凹陷峰值，並吸收周圍溝壁擴散過來的凹陷值
                nextMax[i] = Mathf.Max(maxConcavity[i], sum / (count + 1));
            }
            maxConcavity = nextMax;
        }

        // ==========================================
        // 步驟 5：將凹陷度寫入模型的 UV3 通道 (索引值為 3，對應 Shader Graph 的 UV3)
        // x (R): 環形溝槽/單向最大凹陷度 (0~1，推薦使用)
        // y (G): 平均凹陷度 (0~1)
        // ==========================================
        List<Vector2> uv3Concavity = new List<Vector2>(vertexCount);
        for (int i = 0; i < vertexCount; i++)
        {
            int wIdx = vertexToWelded[i];
            float grooveCavity = Mathf.Clamp01(maxConcavity[wIdx] * ConcavityIntensity);
            float averageCavity = Mathf.Clamp01(avgConcavity[wIdx] * ConcavityIntensity);
            uv3Concavity.Add(new Vector2(grooveCavity, averageCavity));
        }

        mesh.SetUVs(3, uv3Concavity);
    }
}
