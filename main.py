import mesh as m

def main():
    mesh = m.Mesh()
    mesh.loadOBJ("./bunny/reconstruction/bun_zipper.obj")
    mesh.create_edge()
    print("a")


if __name__ == "__main__":
    main()