# 전체 경로 진단

7개 모두 full_path_requested=true, full_path_collected=true. 아래 음수 거리는 명령 로봇 + 초기 식물의 명목 기하 겹침이며 실제 물리 침투량이 아니다.

|경로|상태|완료/종료|끝 시각(s)|고유/저장/생략|상세 잘림|쿼리|시간(s)|
|---|---|---|---:|---|---|---:|---:|
|original02|blocked|True / end_of_path|51.7167|11/11/0|False|967,122|12.71|
|visual02|blocked|True / end_of_path|41.8500|11/11/0|False|723,190|10.43|
|gutter02|blocked|True / end_of_path|41.8500|11/11/0|False|4,223,302|27.93|
|gutter09|blocked|True / end_of_path|43.4500|63/63/0|False|4,249,226|29.39|
|stem01|blocked|True / end_of_path|41.4500|117/64/53|True|4,310,395|36.93|
|stem02|blocked|True / end_of_path|41.8500|75/64/11|True|4,516,489|39.27|
|stem09|blocked|True / end_of_path|43.4500|134/64/70|True|4,436,023|38.03|

클래스별 최초/최악은 상세 64개 저장 제한과 별개로 집계했다. `other/g6`의 실제 바디는 mapping.json 참조.

|경로 / 클래스|최초 시각·단계·쌍·거리(mm)|최악 시각·단계·쌍·거리(mm)|
|---|---|---|
|original02 / glb_plant|15.4750 / insert / g385 → glb_col_TRUSS_Rachis_05 / -0.1781|33.8000 / rise / g410 → glb_col_Tomato_08 / -10.3603|
|visual02 / glb_plant|15.2389 / insert / g390 → glb_col_TRUSS_Rachis_05 / -0.0260|15.3556 / insert / g390 → glb_col_TRUSS_Rachis_05 / -2.0999|
|gutter02 / glb_plant|15.2389 / insert / g390 → glb_col_TRUSS_Rachis_05 / -0.0260|15.3556 / insert / g390 → glb_col_TRUSS_Rachis_05 / -2.0999|
|gutter09 / gutter|7.6833 / preapproach / g410 → gutter_collision_GutterFoldedLip_001 / -0.4116|7.8667 / preapproach / g410 → gutter_collision_GutterFoldedLip_001 / -14.3138|
|gutter09 / glb_plant|13.8083 / entry / g390 → glb_col_Tomato_10 / -0.1653|30.1500 / rise / g410 → glb_col_Tomato_08 / -12.9692|
|stem01 / neighbor_stem|4.1333 / preapproach / g410 → neighbor_stem_collision_18_05 / -0.3220|5.0917 / preapproach / robot_geom_14 → neighbor_stem_collision_18_06 / -45.6883|
|stem01 / other|7.2417 / preapproach / g410 → g6 / -0.0442|7.5500 / preapproach / g410 → g6 / -0.9309|
|stem01 / glb_plant|11.9417 / entry / g387 → glb_col_Tomato_02 / -0.0447|12.9278 / entry / g386 → glb_col_Tomato_02 / -6.5222|
|stem02 / neighbor_stem|4.4333 / preapproach / g410 → neighbor_stem_collision_18_05 / -0.4099|9.9833 / entry / g410 → neighbor_stem_collision_17_02 / -51.9981|
|stem02 / glb_plant|15.2389 / insert / g390 → glb_col_TRUSS_Rachis_05 / -0.0260|15.3556 / insert / g390 → glb_col_TRUSS_Rachis_05 / -2.0999|
|stem09 / neighbor_stem|5.3833 / preapproach / g410 → neighbor_stem_collision_18_05 / -0.1827|11.2250 / entry / robot_geom_14 → neighbor_stem_collision_17_02 / -41.9561|
|stem09 / gutter|7.6833 / preapproach / g410 → gutter_collision_GutterFoldedLip_001 / -0.4116|7.8667 / preapproach / g410 → gutter_collision_GutterFoldedLip_001 / -14.3138|
|stem09 / glb_plant|13.8083 / entry / g390 → glb_col_Tomato_10 / -0.1653|30.1500 / rise / g410 → glb_col_Tomato_08 / -12.9692|
