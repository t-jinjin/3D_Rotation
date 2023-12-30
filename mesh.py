import numpy as np

class Mesh():
    def __init__(self):
        self.v = None
        self.f_by_vID = None
        self.f_by_deID = None
        self.f_by_ueID = None
        self.fID_list_of_v = None
        self.fID_list_of_de = None
        self.fID_list_of_ue = None
        
        self.de_by_vID = None
        self.deID_list_of_v = None
        self.ue_by_vID = None
        self.ueID_list_of_v = None

        self.ueID_list_of_deID = None
        self.deID_list_of_ueID = None



    def loadOBJ(self,filePath):
        numVertices = 0
        numUVs = 0
        numNormals = 0
        numFaces = 0

        vertices = []
        uvs = []
        normals = []
        vertexColors = []
        faceVertIDs = []
        uvIDs = []
        normalIDs = []

        for line in open(filePath, "r"):
            vals = line.split()

            if len(vals) == 0:
                continue

            if vals[0] == "v":
                v = list(map(float, vals[1:4]))
                vertices.append(v)

                if len(vals) == 7:
                    vc = list(map(float, vals[4:7]))
                    vertexColors.append(vc)

                numVertices += 1
            if vals[0] == "vt":
                vt = list(map(float, vals[1:3]))
                uvs.append(vt)
                numUVs += 1
            if vals[0] == "vn":
                vn = list(map(float, vals[1:4]))
                normals.append(vn)
                numNormals += 1
            if vals[0] == "f":
                fvID = []
                uvID = []
                nvID = []
                for f in vals[1:]:
                    w = f.split("/")

                    if numVertices > 0:
                        fvID.append(int(w[0])-1)

                    if numUVs > 0:
                        uvID.append(int(w[1])-1)

                    if numNormals > 0:
                        nvID.append(int(w[2])-1)

                faceVertIDs.append(fvID)
                uvIDs.append(uvID)
                normalIDs.append(nvID)

                numFaces += 1

        print("numVertices: ", numVertices)
        print("numUVs: ", numUVs)
        print("numNormals: ", numNormals)
        print("numFaces: ", numFaces)
        self.v = np.array(vertices)
        self.f_by_vID = faceVertIDs
        self.n_vertex = len(self.v)
        self.n_face = len(self.f_by_vID)

        self.fID_list_of_v = [[] for i in range(self.n_vertex)]
        #faceを全て訪問する
        for fID, face in enumerate(self.f_by_vID):
            self.fID_list_of_v[face[0]].append(fID)
            self.fID_list_of_v[face[1]].append(fID)
            self.fID_list_of_v[face[2]].append(fID)

    def create_edge(self):
        self.de_by_vID = []
        self.fID_list_of_de = []
        #全てのfaceを巡回する
        for fID, face in enumerate(self.f_by_vID):
            self.de_by_vID.append([face[0],face[1]])
            self.de_by_vID.append([face[1],face[2]])
            self.de_by_vID.append([face[2],face[0]])
        n_de = len(self.de_by_vID)
        #uniqueなものに置き換えてみる
        un = np.unique(self.de_by_vID, axis=0)
        n_de_new = len(un)

        if n_de != n_de_new:
            raise ValueError("Invalid Mesh. Directed Edge has duplecated faces.")
        
        #directed_Edgeを全て訪問して，先頭が小さいものになるようにする
        self.ue_by_vID = []
        for deID, de in enumerate(self.de_by_vID):
            if de[0] >= de[1]:
                self.ue_by_vID.append([de[1], de[0]])
            else:
                self.ue_by_vID.append([de[0], de[1]])

        #ueID_listは重複があるので重複をなくす
        self.ue_by_vID, inv = np.unique(
            self.ue_by_vID,
            return_inverse=True
        )
        self.ue_by_vID = self.ue_by_vID.tolist()
        self.n_u_edge = len(self.ue_by_vID)
        # self.ueID_list_of_deIDは配列長さがn_deID，ueIDリストへの行き先を表している
        #つまりinverceそのもの
        self.ueID_list_of_deID = inv.tolist()
        #ではself.deID_list_of_ueIDはどうするか
        self.deID_list_of_ueID = [[] for i in range(self.n_u_edge)]
        for deID, ueID in enumerate(self.ueID_list_of_deID):
            self.deID_list_of_ueID[ueID].append(deID)


        
        
        print("aa")
