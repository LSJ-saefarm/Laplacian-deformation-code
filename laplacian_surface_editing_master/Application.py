import open3d as o3d
import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import math
from Laplacian import Laplac
#1: 일단 과정 자체의 속도를 늘리기
#2:

class SketchApp:
    def __init__(self, mesh):


        self.mesh=mesh


        # We will create a SceneWidget that fills the entire window, and then
        # a label in the lower left on top of the SceneWidget to display the
        # coordinate.
        app = gui.Application.instance
        self.window = app.create_window("Sketch app", 1000, 800)
        # Since we want the label on top of the scene, we cannot use a layout,
        # so we need to manually layout the window's children.
        self.window.set_on_layout(self._on_layout)
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)
        self.info = gui.Label("Sketch App")
        self.info.visible = False
        self.window.add_child(self.info)
        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)

        em = self.window.theme.font_size
        layout = gui.Vert(0, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))
        self.drawingMode = gui.ToggleSwitch("Drawing mode")
        self.drawingMode.set_on_clicked(self._on_drawing_mode)
        layout.add_child(self.drawingMode)
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
        self.window.set_on_key(self._on_key_window)

        self.lineset = []
        self.depth = 1.0
        self.meshMat = rendering.MaterialRecord()
        self.meshMat.shader = "defaultLit"

        self.meshCnt = 0

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.widget3d.frame = r
        pref = self.info.calc_preferred_size(layout_context,
                                             gui.Widget.Constraints())
        self.info.frame = gui.Rect(r.x,
                                   r.get_bottom() - pref.height, pref.width,
                                   pref.height)

    def _on_mouse_widget3d(self, event):
        if not self.drawingMode.is_on:
            return gui.Widget.EventCallbackResult.IGNORED

        if event.type == gui.MouseEvent.Type.BUTTON_DOWN or event.type == gui.MouseEvent.Type.DRAG:

            def depth_callback(depth_image):
                x = event.x - self.widget3d.frame.x
                y = event.y - self.widget3d.frame.y

                Temp=np.asarray(depth_image)[y, x]
                if Temp != math.inf or Temp != -math.inf:
                    print("tlqkf")
                    print(Temp)
                    depth =  Temp
                if len(self.lineset) > 0 and (abs(self.lineset[-1][0] - x) < 1 and abs(self.lineset[-1][1] - y) < 1):
                    return gui.Widget.EventCallbackResult.IGNORED

                if self.depth == 1.0 and depth != 1.0:
                    self.depth = depth
                self.lineset.append([x, y])
                print(x, y)

            self.widget3d.scene.scene.render_to_depth_image(depth_callback)
            return gui.Widget.EventCallbackResult.HANDLED

        elif event.type == gui.MouseEvent.Type.BUTTON_UP:
            if len(self.lineset) > 1 and self.depth != 0: #1로 변경
                lineset = o3d.geometry.LineSet()
                points = []
                lines = []
                for i, p in enumerate(self.lineset):
                    print(self.depth)
                    world = self.widget3d.scene.camera.unproject(
                        p[0], p[1], self.depth, self.widget3d.frame.width,
                        self.widget3d.frame.height)

                    print(f"zzzz{world}")

                    '''  Proj=self.widget3d.scene.camera.get_projection_matrix()
                    view_mtx = self.widget3d.scene.camera.get_view_matrix()
                    proj_mtx = Proj
                    coord3d = [world[0], world[1], world[2], 1]
                    print(coord3d)
                    coord = np.matmul(proj_mtx, np.matmul(view_mtx, coord3d))

                    x = ((coord[0] / coord[3]) / 2.0 + 0.5) * self.widget3d.frame.width
                    y = 800-((coord[1] / coord[3]) / 2.0 + 0.5) * self.widget3d.frame.height

                    print(x, y)
                    print(p)
                    '''

                    points.append(world)

                    '''print(np.asarray(Proj))
                    print(world, p)
                    print("-------------")'''
                    lines.append([i, i + 1])

                lines[-1][1] = 0 #closed curve
                lineset.points = o3d.utility.Vector3dVector(np.asarray(points))
                lineset.lines = o3d.utility.Vector2iVector(np.asarray(lines))
                view_matrix = self.widget3d.scene.camera.get_view_matrix()
                initial_direc = np.array([0, 0, -1, 1])
                direc = np.matmul(initial_direc, view_matrix)
                direc = np.array([direc[0] / direc[3], direc[1] / direc[3], direc[2] / direc[3]])


                proj_mtx=self.widget3d.scene.camera.get_projection_matrix()

                #-----여기서 이제 함수 호출하면 됨--------------
                View={'sketch': self.lineset, '3dsketch': lineset, 'view_matrix': view_matrix, 'proj_matrix': proj_mtx, 'direction': direc,
                      'framesize': (self.widget3d.frame.width,
                        self.widget3d.frame.height), 'widget3d':self} #각각 원본 sketch(list),
                # 공중에 떠있는 sketch(o3d.lineset), view_matrix(numpy), direc(최종 camera direction), numpy
                '''if event.key == ctrl_key and event.type == gui.KeyEvent.Type.UP:
                    view_mtx = self.widget3d.scene.camera.get_view_matrix()
                    proj_mtx = self.widget3d.scene.camera.get_projection_matrix()
                    coord3d = [0, 0, 0, 1]
                    coord = np.matmul(proj_mtx, np.matmul(view_mtx, coord3d))
                    x = ((coord[0] / coord[3]) / 2.0 + 0.5) * self.widget3d.frame.width
                    y = ((coord[1] / coord[3]) / 2.0 + 0.5) * self.widget3d.frame.height
                    print(x, y)
                    return gui.Widget.EventCallbackResult.HANDLED'''


                mesh = Laplac(self.mesh, View) #mesh(open3d)를 return
                if mesh != None:
                    self.mesh=mesh
                    self.widget3d.scene.clear_geometry()
                    self.widget3d.scene.add_geometry("Extrude" + str(self.meshCnt), mesh, self.meshMat)
                    self.meshCnt += 1

            self.lineset = []
            self.depth = 1.0
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED

    def _on_key_window(self, event):
        ctrl_key = 258
        if event.key == ctrl_key and event.type == gui.KeyEvent.Type.UP:
            if len(self.lineset) > 0 and self.depth != 0:
                lineset = o3d.geometry.LineSet()
                points = []
                lines = []
                for i, p in enumerate(self.lineset):
                    world = self.widget3d.scene.camera.unproject(
                        p[0], p[1], self.depth, self.widget3d.frame.width,
                        self.widget3d.frame.height)
                    points.append(world)
                    lines.append([i, i + 1])
                lines[-1][1] = 0
                lineset.points = o3d.utility.Vector3dVector(np.asarray(points))
                lineset.lines = o3d.utility.Vector2iVector(np.asarray(lines))

                view_matrix = self.widget3d.scene.camera.get_view_matrix()
                initial_direc = np.array([0, 0, -1, 1])
                direc = np.matmul(initial_direc, view_matrix)
                direc = np.array([direc[0] / direc[3], direc[1] / direc[3], direc[2] / direc[3]])
                mesh = self.create_mesh(lineset, direc)
                self.widget3d.scene.add_geometry("Extrude" + str(self.meshCnt), mesh, self.meshMat)
                self.meshCnt += 1
            self.lineset = []
            self.depth = 1.0
            return True
        return False

    #<<Hierarchy>>
    '''window: 걍 윈도우(KeyEvent 등록 가능)
    ㄴwidget3d: 마우스로 뷰 돌리는 위젯(MouseEvent 등록됨)
    ㄴlayout: 좌측 상단에 까만 사각형(UI)
       ㄴdrawingMode: 토글 버튼(켜졌을 때 뷰 컨트롤 안 받고 마우스 입력으로 드로잉 받음
    
    <<함수 설명>>
    _on_layout: 딱히 볼 필요 없음
    _on_mouse_widget3d: widget3d의 마우스이벤트. 조건 해당되면 detph_callback 호출. event 객체는 화면 상의 x, y 좌표를 저장. depth는 모니터와 수직인 z 좌표를 얻음(물체와 부딪힌 부분으로부터) 마우스 이벤트에 따라 2가지 케이스로 나누어서 처리.
      1) 버튼 다운 || 드래그
    마우스의 (x, y) 위치만 저장. self.lineset은 3D가 아니라 2D (x, y) 위치만 배열로 저장함. depth가 없으면 depth를 업데이트 해줌. 스케치를 하는 동안 한 번이라도 물체와 부딪혀야 depth가 생기는데, 그렇지 않은 경우 depth가 없어서 스케치 인식이 안 됨. depth 정보는 처음 부딪힌 부분으로 세팅됨.
      2) 버튼 업
    이제 스케치 끝난다는 뜻이므로 self.lineset의 line을 closed curve로 만듦.(이 부분 없애려면 95줄 수정.(lines[-1][1]=0 대신 lines.pop()) 그 다음 o3d의 Lineset 객체를 생성. self.lineset에 있던 좌표를 꺼내와서 depth 정보를 이용해서 3D 좌표로 변환해서 points에 넣고, 차례로 이어서 lines에 넣음. 2d->3d가 unprojection이기 때문에 depth값에 따라서 위치가 당연히 달라지는데, 렌더링을 parallel view로 하지 않고 projective view로 하기 때문에 depth 영향을 좀 받는 것 같음. (깊게 공부 안 해도 될듯) direc은 뷰 디렉션 구하는 거고, create_mesh는 extrude한 메쉬 만드는 부분이니까 필요없으면 삭제하고 lineset 정보만 쓰면 됨.
    
    _on_key_window: 삭제하는 거 깜박함. 없애도 됨. 44줄도 같이 없애주고. 이게 윈도우 키 이벤트니까 다른 키 이벤트 ord("S") 같은 거 등록하려면 44줄 놔두고 이 함수 수정해서 쓰면 됨.
    '''

    def create_mesh(self, lineset, direc):
        lineLen = len(lineset.points)
        mesh = o3d.geometry.TriangleMesh()

        points = np.asarray(lineset.points)
        center = np.sum(points, axis=0) / len(points)
        vertices = np.append(points, [center], axis=0)
        vertices_back = vertices.copy()
        for v in vertices:
            v -= 0.1 * direc
        for v in vertices_back:
            v += 0.1 * direc

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
        # o3d.visualization.draw_geometries([mesh])
        return mesh

    def _on_drawing_mode(self, is_on):
        if is_on:
            # self.widget3d.enabled = False
            self.widget3d.set_view_controls(self.widget3d.ROTATE_IBL)
            self.lineset = []
            self.depth = 1.0
        else:
            # self.widget3d.enabled = True
            self.widget3d.scene.show_skybox(False)
            self.widget3d.set_view_controls(self.widget3d.ROTATE_CAMERA)


def main():
    app = gui.Application.instance
    app.initialize()
    mesh = o3d.io.read_triangle_mesh("meshes/00288_processed.ply")
    mesh.compute_vertex_normals()
    ex = SketchApp(mesh)

    app.run()


if __name__ == "__main__":
    main()
# %%