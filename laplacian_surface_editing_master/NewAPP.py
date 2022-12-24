import open3d as o3d
import numpy as np
from skimage import measure
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import math
from Laplacian import Laplac
import copy

Read_ref = "./meshes/00288_sdf.ply"
Proc_ref = "./meshes/00288_processed.ply"
Prev_ref = "./meshes/00288_previous.ply"
Sdf_ref = './meshes/00288_sdf.npy'

class SketchApp:
    def __init__(self, mesh, mesh_sdf):
        app = gui.Application.instance
        self.window = app.create_window("Sketch app", 1000, 800)
        self.window.set_on_layout(self._on_layout)
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)
        self.info = gui.Label("Sketch App")
        self.info.visible = False
        self.window.add_child(self.info)
        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)

        em = self.window.theme.font_size
        layout = gui.Vert(0, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))

        self.actionRadio = gui.RadioButton(gui.RadioButton.VERT)
        self.actionRadio.set_items(["View", "Hole filling", "Hole creation", "Mesh deformation(Rough)", "Mesh deformation(Detail)"])
        self.actionRadio.set_on_selection_changed(self._on_action_radio)
        self.undoButton = gui.Button("UNDO")
        self.undoButton.set_on_clicked(self._on_undo_clicked)
        self.resetButton = gui.Button("RESET")
        self.resetButton.set_on_clicked(self._on_reset_clicked)
        layout.add_child(self.actionRadio)
        layout.add_fixed(1.0 * em)
        layout.add_child(self.undoButton)
        layout.add_fixed(0.5 * em)
        layout.add_child(self.resetButton)
        self.window.add_child(layout)

        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        # Point size is in native pixels, but "pixel" means different things to
        # different platforms (macOS, in particular), so multiply by Window scale
        # factor.
        mat.point_size = 3 * self.window.scaling
        self.widget3d.scene.add_geometry("Mesh", mesh, mat)

        bounds = self.widget3d.scene.bounding_box
        center = bounds.get_center()
        self.widget3d.setup_camera(60, bounds, center)
        self.widget3d.look_at(center, center - [0, 0, -1], [0, 1, 0])
        self.widget3d.set_on_mouse(self._on_mouse_widget3d)
        self.widget3d.set_on_key(self._on_key_widget3d)

        self.depth = 1.0
        self.mesh = mesh
        self.meshMat = rendering.MaterialRecord()
        self.meshMat.shader = "defaultLit"

        self.line3D = o3d.geometry.LineSet()
        self.lineMat = rendering.MaterialRecord()
        self.lineMat.shader = "unlitLine"
        self.lineMat.line_width = 3

        self.meshCnt = 0

        self.num = 64
        min_bound = [-.5, -.5, -.5]
        max_bound = [.5, .5, .5]
        xyz_range = np.linspace(min_bound, max_bound, num=self.num)
        self.query_points = np.stack(np.meshgrid(*xyz_range.T), axis=-1).astype(np.float32)

        self.mesh_sdf = mesh_sdf
        self.mesh_sdf_prev = mesh_sdf

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.widget3d.frame = r
        pref = self.info.calc_preferred_size(layout_context,
                                             gui.Widget.Constraints())
        self.info.frame = gui.Rect(r.x,
                                   r.get_bottom() - pref.height, pref.width,
                                   pref.height)

    def _on_mouse_widget3d(self, event):
        if self.actionRadio.selected_index == 0:
            return gui.Widget.EventCallbackResult.IGNORED

        def depth_callback(depth_image):
            x = event.x - self.widget3d.frame.x
            y = event.y - self.widget3d.frame.y
            if len(self.line2D) > 0 and (abs(self.line2D[-1][0] - x) < 1 and abs(self.line2D[-1][1] - y) < 1):
                return
            depth = np.asarray(depth_image)[y, x]
            if self.depth == 1.0 and depth != 1.0 and abs(depth - self.widget3d.scene.camera.get_near()) >= 0.001:
                self.depth = depth

            self.line2D.append([x, y])

        def line_add():
            near = self.widget3d.scene.camera.get_near()
            x = event.x - self.widget3d.frame.x
            y = event.y - self.widget3d.frame.y
            world = self.widget3d.scene.camera.unproject(
                x, y, near, self.widget3d.frame.width,
                self.widget3d.frame.height)
            points = np.asarray(self.line3D.points)
            points = np.append(points, [world], axis=0)
            self.line3D.points = o3d.utility.Vector3dVector(points)
            length = len(np.asarray(self.line3D.points))
            if length > 1:
                lines = np.asarray(self.line3D.lines)
                lines = np.append(lines, [[length - 2, length - 1]], axis=0)
                self.line3D.lines = o3d.utility.Vector2iVector(lines)
                colors = np.asarray(self.line3D.colors)
                colors = np.append(colors, [[1, 0, 1]], axis=0)
                self.line3D.colors = o3d.utility.Vector3dVector(colors)

        if event.is_modifier_down(gui.KeyModifier.CTRL):
            if event.type == gui.MouseEvent.Type.BUTTON_UP:
                if len(self.line3D.points) == 0:
                    if self.widget3d.scene.has_geometry("Line"):
                        self.widget3d.scene.remove_geometry("Line")
                        self.line3D.clear()
                    self.widget3d.scene.add_geometry("Line", self.line3D, self.lineMat)
                    self.widget3d.scene.scene.render_to_depth_image(depth_callback)
                    line_add()
                    return gui.Widget.EventCallbackResult.HANDLED
                else:
                    self.widget3d.scene.scene.render_to_depth_image(depth_callback)
                    line_add()
                    self.widget3d.scene.remove_geometry("Line")
                    self.widget3d.scene.add_geometry("Line", self.line3D, self.lineMat)
                    return gui.Widget.EventCallbackResult.HANDLED
            return gui.Widget.EventCallbackResult.IGNORED

        elif event.type == gui.MouseEvent.Type.BUTTON_DOWN:
            self.widget3d.scene.add_geometry("Line", self.line3D, self.lineMat)
            self.widget3d.scene.scene.render_to_depth_image(depth_callback)
            line_add()
            return gui.Widget.EventCallbackResult.HANDLED

        elif event.type == gui.MouseEvent.Type.DRAG:
            self.widget3d.scene.scene.render_to_depth_image(depth_callback)
            line_add()
            self.widget3d.scene.remove_geometry("Line")
            self.widget3d.scene.add_geometry("Line", self.line3D, self.lineMat)
            return gui.Widget.EventCallbackResult.HANDLED

        elif event.type == gui.MouseEvent.Type.BUTTON_UP:
            if len(self.line2D) < 3 or self.depth == 1.0 or abs(
                    self.depth - self.widget3d.scene.camera.get_near()) < 0.001:
                self.line2D = []
                self.depth = 1.0
                self.line3D.clear()
                if self.widget3d.scene.has_geometry("Line"):
                    self.widget3d.scene.remove_geometry("Line")
                return gui.Widget.EventCallbackResult.IGNORED

            points = np.asarray(self.line3D.points)
            lines = np.asarray(self.line3D.lines)
            lines = np.append(lines, [[len(points) - 1, 0]], axis=0)
            self.line3D.lines = o3d.utility.Vector2iVector(lines)
            colors = np.asarray(self.line3D.colors)
            colors = np.append(colors, [[1, 0, 1]], axis=0)
            self.line3D.colors = o3d.utility.Vector3dVector(colors)

            for i, p in enumerate(self.line2D):
                world = self.widget3d.scene.camera.unproject(
                    p[0], p[1], self.depth, self.widget3d.frame.width,
                    self.widget3d.frame.height)
                points[i] = world
            self.line3D.points = o3d.utility.Vector3dVector(points)
            self.widget3d.scene.remove_geometry("Line")
            self.widget3d.scene.add_geometry("Line", self.line3D, self.lineMat)

            view_matrix = self.widget3d.scene.camera.get_view_matrix()
            initial_direc = np.array([0, 0, -1, 1])
            direc = np.matmul(initial_direc, view_matrix)
            direc = np.array([direc[0] / direc[3], direc[1] / direc[3], direc[2] / direc[3]])

            action = self.actionRadio.selected_index
            if action == 1:
                box = self.create_mesh(self.line3D, direc)
                new_mesh = self.hole_filling(self.mesh, box)
                self.widget3d.scene.remove_geometry("Mesh")
                self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                self.mesh = new_mesh
            elif action == 2:
                box = self.create_mesh(self.line3D, direc)
                new_mesh = self.hole_creation(self.mesh, box)
                self.widget3d.scene.remove_geometry("Mesh")
                self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                self.mesh = new_mesh
            elif action == 3:
                proj_mtx = self.widget3d.scene.camera.get_projection_matrix()
                View = {'sketch': self.line2D, '3dsketch': self.line3D, 'view_matrix': view_matrix,
                        'proj_matrix': proj_mtx, 'direction': direc,
                        'framesize': (self.widget3d.frame.width,
                                      self.widget3d.frame.height), 'widget3d': self, 'Detail': 4}  # 각각 원본 sketch(list),
                # 공중에 떠있는 sketch(o3d.lineset), view_matrix(numpy), direc(최종 camera direction), numpy
                print(View)
                new_mesh = Laplac(self.mesh, View)  # mesh(open3d)를 return
                if new_mesh != None:
                    new_mesh.paint_uniform_color([0.5, 0.4, 0.3])
                    self.widget3d.scene.remove_geometry("Mesh")
                    self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                    o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                    o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                    self.mesh = new_mesh
                    self.mesh_sdf = self.calculate_sdf(new_mesh)
            elif action == 4:
                proj_mtx = self.widget3d.scene.camera.get_projection_matrix()
                View = {'sketch': self.line2D, '3dsketch': self.line3D, 'view_matrix': view_matrix,
                        'proj_matrix': proj_mtx, 'direction': direc,
                        'framesize': (self.widget3d.frame.width,
                                      self.widget3d.frame.height), 'widget3d': self, 'Detail': 2}  # 각각 원본 sketch(list),
                # 공중에 떠있는 sketch(o3d.lineset), view_matrix(numpy), direc(최종 camera direction), numpy
                new_mesh = Laplac(self.mesh, View)  # mesh(open3d)를 return
                if new_mesh != None:
                    new_mesh.paint_uniform_color([0.5, 0.4, 0.3])
                    self.widget3d.scene.remove_geometry("Mesh")
                    self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                    o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                    o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                    self.mesh = new_mesh
                    self.mesh_sdf = self.calculate_sdf(new_mesh)
            self.line2D = []
            self.depth = 1.0
            self.line3D.clear()
            self.widget3d.scene.remove_geometry("Line")
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED

    def _on_key_widget3d(self, event):
        ctrl_key = 258
        if event.key == ctrl_key and event.type == gui.KeyEvent.Type.UP:
            if len(self.line2D) < 3 or self.depth == 1.0 or abs(
                    self.depth - self.widget3d.scene.camera.get_near()) < 0.001:
                self.line2D = []
                self.depth = 1.0
                self.line3D.clear()
                if self.widget3d.scene.has_geometry("Line"):
                    self.widget3d.scene.remove_geometry("Line")
                return gui.Widget.EventCallbackResult.IGNORED

            points = np.asarray(self.line3D.points)
            lines = np.asarray(self.line3D.lines)
            lines = np.append(lines, [[len(points) - 1, 0]], axis=0)
            self.line3D.lines = o3d.utility.Vector2iVector(lines)
            colors = np.asarray(self.line3D.colors)
            colors = np.append(colors, [[1, 0, 1]], axis=0)
            self.line3D.colors = o3d.utility.Vector3dVector(colors)

            for i, p in enumerate(self.line2D):
                world = self.widget3d.scene.camera.unproject(
                    p[0], p[1], self.depth, self.widget3d.frame.width,
                    self.widget3d.frame.height)
                points[i] = world
            self.line3D.points = o3d.utility.Vector3dVector(points)
            self.widget3d.scene.remove_geometry("Line")
            self.widget3d.scene.add_geometry("Line", self.line3D, self.lineMat)

            view_matrix = self.widget3d.scene.camera.get_view_matrix()
            initial_direc = np.array([0, 0, -1, 1])
            direc = np.matmul(initial_direc, view_matrix)
            direc = np.array([direc[0] / direc[3], direc[1] / direc[3], direc[2] / direc[3]])

            action = self.actionRadio.selected_index
            if action == 1:
                box = self.create_mesh(self.line3D, direc)
                new_mesh = self.hole_filling(self.mesh, box)
                self.widget3d.scene.remove_geometry("Mesh")
                self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                self.mesh = new_mesh
            elif action == 2:
                box = self.create_mesh(self.line3D, direc)
                new_mesh = self.hole_creation(self.mesh, box)
                self.widget3d.scene.remove_geometry("Mesh")
                self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                self.mesh = new_mesh
            elif action == 3:
                proj_mtx = self.widget3d.scene.camera.get_projection_matrix()
                View = {'sketch': self.line2D, '3dsketch': self.line3D, 'view_matrix': view_matrix,
                        'proj_matrix': proj_mtx, 'direction': direc,
                        'framesize': (self.widget3d.frame.width,
                                      self.widget3d.frame.height), 'widget3d': self, 'Detail': 4}  # 각각 원본 sketch(list),
                # 공중에 떠있는 sketch(o3d.lineset), view_matrix(numpy), direc(최종 camera direction), numpy
                print(View)
                new_mesh = Laplac(self.mesh, View)  # mesh(open3d)를 return
                if new_mesh != None:
                    new_mesh.paint_uniform_color([0.5, 0.4, 0.3])
                    self.widget3d.scene.remove_geometry("Mesh")
                    self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                    o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                    o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                    self.mesh = new_mesh
                    self.mesh_sdf = self.calculate_sdf(new_mesh)
            elif action == 4:
                proj_mtx = self.widget3d.scene.camera.get_projection_matrix()
                View = {'sketch': self.line2D, '3dsketch': self.line3D, 'view_matrix': view_matrix,
                        'proj_matrix': proj_mtx, 'direction': direc,
                        'framesize': (self.widget3d.frame.width,
                                      self.widget3d.frame.height), 'widget3d': self, 'Detail': 2}  # 각각 원본 sketch(list),
                # 공중에 떠있는 sketch(o3d.lineset), view_matrix(numpy), direc(최종 camera direction), numpy
                new_mesh = Laplac(self.mesh, View)  # mesh(open3d)를 return
                if new_mesh != None:
                    new_mesh.paint_uniform_color([0.5, 0.4, 0.3])
                    self.widget3d.scene.remove_geometry("Mesh")
                    self.widget3d.scene.add_geometry("Mesh", new_mesh, self.meshMat)
                    o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
                    o3d.io.write_triangle_mesh(Proc_ref, new_mesh)
                    self.mesh = new_mesh
                    self.mesh_sdf = self.calculate_sdf(new_mesh)

            self.line2D = []
            self.depth = 1.0
            self.line3D.clear()
            self.widget3d.scene.remove_geometry("Line")
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED

    def create_mesh(self, lineset, direc):
        lineLen = len(lineset.points)
        mesh = o3d.geometry.TriangleMesh()

        points = np.asarray(lineset.points)
        center = np.sum(points, axis=0) / len(points)
        vertices = np.append(points, [center], axis=0)
        vertices_back = vertices.copy()
        for v in vertices:
            v -= 0.2 * direc
        for v in vertices_back:
            v += 0.2 * direc

        triangles = np.asarray(mesh.triangles)
        centerIdx = lineLen
        for i in range(centerIdx - 1):
            triangles = np.append(triangles, [[i, i + 1, centerIdx]], axis=0)
        triangles = np.append(triangles, [[centerIdx - 1, 0, centerIdx]], axis=0)

        triangles_back = np.asarray(mesh.triangles)
        centerIdx_back = 2 * len(vertices) - 1
        for i in range(len(vertices), centerIdx_back - 1):
            triangles_back = np.append(triangles_back, [[i + 1, i, centerIdx_back]], axis=0)
        triangles_back = np.append(triangles_back, [[len(vertices), centerIdx_back - 1, centerIdx_back]], axis=0)

        triangles_side = np.asarray(mesh.triangles)
        for i in range(lineLen - 1):
            triangles_side = np.append(triangles_side, [[i + 1, i, i + lineLen + 1]], axis=0)
            triangles_side = np.append(triangles_side, [[i + 1, i + lineLen + 1, i + lineLen + 2]], axis=0)
        triangles_side = np.append(triangles_side, [[0, centerIdx - 1, lineLen * 2]], axis=0)
        triangles_side = np.append(triangles_side, [[0, lineLen * 2, lineLen + 1]], axis=0)

        mesh.vertices = o3d.utility.Vector3dVector(np.append(vertices, vertices_back, axis=0))
        mesh.triangles = o3d.utility.Vector3iVector(
            np.append(np.append(triangles, triangles_back, axis=0), triangles_side, axis=0))
        mesh.compute_vertex_normals()

        return mesh

    def hole_filling(self, mesh, box):
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        box_t = o3d.t.geometry.TriangleMesh.from_legacy(box)

        inter_t = mesh_t.boolean_intersection(box_t)
        inter = inter_t.to_legacy()
        inter.compute_vertex_normals()

        pcd = o3d.geometry.PointCloud()
        pcd.points = inter.vertices
        alpha = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.05)
        alpha.compute_vertex_normals()

        box_sdf = self.calculate_sdf(alpha)

        sdf = np.zeros(shape=(self.num, self.num, self.num))
        for i in range(self.num):
            for j in range(self.num):
                for k in range(self.num):
                    sdf[i][j][k] = min(self.mesh_sdf[i][j][k], box_sdf[i][j][k])
        self.mesh_sdf_prev = self.mesh_sdf
        self.mesh_sdf = sdf

        spacing = 1 / (self.num - 1)
        verts, faces, normals, values = measure.marching_cubes(sdf, 0.0005, spacing=(spacing, spacing, spacing))

        new_mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts), o3d.utility.Vector3iVector(faces))
        new_mesh.compute_vertex_normals()
        new_mesh = new_mesh.translate(-1 * new_mesh.get_center())
        new_mesh.paint_uniform_color([0.5, 0.4, 0.3])

        triangle_clusters, cluster_n_triangles, cluster_area = (new_mesh.cluster_connected_triangles())
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        cluster_area = np.asarray(cluster_area)
        max_triangles = np.max(cluster_n_triangles)
        triangles_to_remove = cluster_n_triangles[triangle_clusters] != max_triangles
        new_mesh.remove_triangles_by_mask(triangles_to_remove)

        return new_mesh

    def hole_creation(self, mesh, box):
        box_sdf = self.calculate_sdf(box)

        sdf = np.zeros(shape=(self.num, self.num, self.num))
        for i in range(self.num):
            for j in range(self.num):
                for k in range(self.num):
                    if self.mesh_sdf[i][j][k] < 0 and box_sdf[i][j][k] < 0:
                        sdf[i][j][k] = -box_sdf[i][j][k]
                    else:
                        sdf[i][j][k] = self.mesh_sdf[i][j][k]
        self.mesh_sdf_prev = self.mesh_sdf
        self.mesh_sdf = sdf

        spacing = 1 / self.num
        verts, faces, normals, values = measure.marching_cubes(sdf, 0, spacing=(spacing, spacing, spacing))

        new_mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts), o3d.utility.Vector3iVector(faces))
        new_mesh.compute_vertex_normals()
        new_mesh = new_mesh.translate(-1 * new_mesh.get_center())
        new_mesh.paint_uniform_color([0.5, 0.4, 0.3])

        triangle_clusters, cluster_n_triangles, cluster_area = (new_mesh.cluster_connected_triangles())
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        cluster_area = np.asarray(cluster_area)
        max_triangles = np.max(cluster_n_triangles)
        triangles_to_remove = cluster_n_triangles[triangle_clusters] != max_triangles
        new_mesh.remove_triangles_by_mask(triangles_to_remove)

        return new_mesh

    def calculate_sdf(self, mesh):
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        scene = o3d.t.geometry.RaycastingScene()
        _ = scene.add_triangles(mesh_t)
        signed_distance = scene.compute_signed_distance(self.query_points)
        mesh_sdf = signed_distance.numpy()
        sdf = copy.deepcopy(mesh_sdf)
        for i in range(self.num):
            for j in range(self.num):
                for k in range(self.num):
                    sdf[j][i][k] = mesh_sdf[i][j][k]
        return sdf

    def _on_action_radio(self, idx):
        if idx == 0:
            self.widget3d.scene.show_skybox(False)
            self.widget3d.set_view_controls(self.widget3d.ROTATE_CAMERA)
        else:
            self.widget3d.set_view_controls(self.widget3d.ROTATE_IBL)
            self.line2D = []
            self.depth = 1.0

    def _on_undo_clicked(self):
        self.mesh = o3d.io.read_triangle_mesh(Prev_ref)
        o3d.io.write_triangle_mesh(Proc_ref, self.mesh)
        o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
        self.mesh_sdf = self.mesh_sdf_prev
        self.widget3d.scene.remove_geometry("Mesh")
        self.widget3d.scene.add_geometry("Mesh", self.mesh, self.meshMat)

    def _on_reset_clicked(self):
        self.mesh = o3d.io.read_triangle_mesh(Read_ref)
        o3d.io.write_triangle_mesh(Proc_ref, self.mesh)
        o3d.io.write_triangle_mesh(Prev_ref, self.mesh)
        self.mesh_sdf = np.load(Sdf_ref)
        self.widget3d.scene.remove_geometry("Mesh")
        self.widget3d.scene.add_geometry("Mesh", self.mesh, self.meshMat)


def main():
    app = gui.Application.instance
    app.initialize()
    mesh = o3d.io.read_triangle_mesh(Read_ref)
    mesh.paint_uniform_color([0.5, 0.4, 0.3])
    o3d.io.write_triangle_mesh(Proc_ref, mesh)
    o3d.io.write_triangle_mesh(Prev_ref, mesh)
    mesh.compute_vertex_normals()
    mesh_sdf = np.load(Sdf_ref)
    ex = SketchApp(mesh, mesh_sdf)

    app.run()


if __name__ == "__main__":
    main()

