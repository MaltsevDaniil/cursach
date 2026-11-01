# REFERENCES — проверенная библиография

Дата проверки: 22.10.2026. Область: реактивная децентрализованная навигация, формации и локальное предотвращение столкновений без обучения. Идентификаторы R01–R11 используются в LITERATURE_REVIEW.md.

## Как читать статусы

**Текст** — доступен текст первичной публикации или авторской версии; это не означает воспроизведения всех результатов. **Карточка** — существование и указанные реквизиты подтверждены издателем, автором или университетским репозиторием, но полный текст здесь не изучен. Оценки трудоёмкости и пригодности в обзоре — инженерный вывод, а не численный результат статей. DOI не восстанавливались по памяти, если не удалось проверить; URL достаточен для идентификации. Это целевой, а не систематический обзор всей литературы до 2026 года.

## Основной список

### R01 — APF

Oussama Khatib. **Real-Time Obstacle Avoidance for Manipulators and Mobile Robots**. 1986. *The International Journal of Robotics Research*, 5(1), 90–98.

- DOI: [10.1177/027836498600500106](https://doi.org/10.1177/027836498600500106).
- Проверка: [издатель](https://journals.sagepub.com/doi/pdf/10.1177/027836498600500106), [список публикаций Stanford](https://khatib.stanford.edu/publications-journals.html).
- Авторская копия: [PDF](https://khatib.stanford.edu/publications/pdfs/Khatib_1986_IJRR.pdf).
- Статус: **карточка/аннотация**; непосредственное открытие PDF завершилось тайм-аутом. Это журнальная статья, не одноимённая глава с другим DOI.
- Использование: основа притяжения к цели и отталкивания от препятствий; не источник гарантии выхода из произвольных ловушек.

### R02 — Boids

Craig W. Reynolds. **Flocks, Herds and Schools: A Distributed Behavioral Model**. 1987. *Computer Graphics*, 21(4), SIGGRAPH ’87, 25–34.

- DOI: [10.1145/37401.37406](https://doi.org/10.1145/37401.37406).
- Проверка: [страница автора](https://www.red3d.com/cwr/papers/1987/boids.html), [полный текст у University of Toronto](https://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/), [ACM](https://doi.org/10.1145/37401.37406).
- Статус: **текст и авторская карточка**.
- Использование: локальные правила separation, alignment, cohesion. Исходная работа посвящена поведенческой анимации; её нельзя цитировать как доказательство безопасности физических роботов.

### R03 — управляемый flocking

Reza Olfati-Saber. **Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory**. 2006. *IEEE Transactions on Automatic Control*, 51(3), 401–420.

- DOI: [10.1109/TAC.2005.864190](https://doi.org/10.1109/TAC.2005.864190).
- Проверенная первичная авторская версия: [Technical Report CIT-CDS 2004-005, 22 June 2004](https://www.seas.ucla.edu/coopcontrol/papers/flocking_tac04b.pdf).
- Статус: **текст авторского отчёта 2004 года**. Реквизиты журнальной версии и DOI согласуются с библиографическими записями, но окончательная журнальная версия отдельно не прочитана. Не смешивать год отчёта и год журнала; эквивалентность всех формул версий не проверена.
- Использование: управление расстояниями и скоростями соседей, общее направление миграции, виртуальные агенты препятствий; в авторском тексте есть split/rejoin и squeezing. Для бакалаврского проекта достаточно понять структуру контроллера, не воспроизводя доказательства.

### R04 — формации как поведенческий контроллер

Tucker Balch; Ronald C. Arkin. **Behavior-Based Formation Control for Multirobot Teams**. 1998. *IEEE Transactions on Robotics and Automation*, 14(6), 926–939.

- URL: [опубликованный PDF лаборатории Georgia Tech](https://sites.cc.gatech.edu/ai/robot-lab/online-publications/ieee/00736776.pdf).
- Дополнительная проверка: [IEEE, document 736776](https://ieeexplore.ieee.org/document/736776/).
- Статус: **текст опубликованной статьи**. DOI здесь не приводится: прямая карточка IEEE закрыта проверкой JavaScript; для идентификации используется проверенный PDF.
- Использование: совмещение удержания строя, движения к цели и обхода опасностей. Разные схемы опорных точек требуют разного объёма информации.

### R05 — circulation / gyroscopic

Dong Eui Chang; Shawn C. Shadden; Jerrold E. Marsden; Reza Olfati-Saber. **Collision Avoidance for Multiple Agent Systems**. 2003. *Proceedings of the 42nd IEEE Conference on Decision and Control*, vol. 1, 539–543.

- Проверка: [репозиторий KAIST](https://pure.kaist.ac.kr/en/publications/collision-avoidance-for-multiple-agent-systems/).
- Ссылка на первичный PDF: [конференционный архив](https://congres.cran.univ-lorraine.fr/2003/CDC%202003/pdffiles/papers/TuM02.3.pdf).
- Статус: **университетская карточка/аннотация**; PDF при открытии недоступен по тайм-ауту; DOI не проверен.
- Использование: сочетание скалярных потенциалов и гироскопических сил для агентов и препятствий. Это предшественник идеи непотенциального обхода, но не утверждение, что формула пользователя совпадает с этой работой.

### R06 — velocity obstacles

Paolo Fiorini; Zvi Shiller. **Motion Planning in Dynamic Environments Using Velocity Obstacles**. 1998. *The International Journal of Robotics Research*, 17(7), 760–772.

- DOI: [10.1177/027836499801700706](https://doi.org/10.1177/027836499801700706).
- Проверка: [издатель](https://journals.sagepub.com/doi/10.1177/027836499801700706).
- Первичный текст: [авторская университетская копия](https://www.ariel.ac.il/sites/shiller/papers/fiorini98motion.pdf).
- Статус: **карточка/аннотация и найденная авторская копия**; детальная реализация по PDF в этой работе не воспроизводилась.
- Использование: исключение скоростей, ведущих к прогнозируемому столкновению; модель движения препятствия на горизонте имеет значение.

### R07 — ORCA

Jur van den Berg; Stephen J. Guy; Ming Lin; Dinesh Manocha. **Reciprocal n-body Collision Avoidance**. 2011. In: Cédric Pradalier, Roland Siegwart, Gerhard Hirzinger (eds.), *Robotics Research: The 14th International Symposium ISRR*, Springer Tracts in Advanced Robotics, vol. 70, 3–19.

- URL: [страница проекта авторов](https://gamma.cs.unc.edu/ORCA/), [полный текст](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf).
- Статус: **текст и авторская библиографическая карточка**; DOI отдельно не проверен.
- Использование: взаимное уклонение через ограничения на скорость и разделение ответственности; ближайшая допустимая скорость к желаемой. Не путать с RVO 2008 или статьёй Snape et al. о differential-drive.
- Условия переноса: голономная модель, корректно наблюдаемые положения/скорости, совместимый протокол у других агентов. Отсутствие необходимости обмениваться сообщениями не означает отсутствия необходимости наблюдать соседей.

### R08 — DWA

Dieter Fox; Wolfram Burgard; Sebastian Thrun. **The Dynamic Window Approach to Collision Avoidance**. 1997. *IEEE Robotics & Automation Magazine*, 4(1).

- DOI: [10.1109/100.580977](https://doi.org/10.1109/100.580977).
- Проверка: [авторская страница University of Washington](https://rse-lab.cs.washington.edu/abstracts/colli-ieee.abstract.html), [IEEE](https://ieeexplore.ieee.org/document/580977/).
- Статус: **авторская аннотация и библиографическая карточка**.
- Использование: выбор достижимых линейной/угловой скоростей с проверкой возможности остановки. Полезен при переходе к кинематике колёсной платформы; не обязательный компонент первого голономного симулятора.

### R09 — safety filter / CBF

Li Wang; Aaron D. Ames; Magnus Egerstedt. **Safety Barrier Certificates for Collisions-Free Multirobot Systems**. 2017. *IEEE Transactions on Robotics*, 33(3), 661–674.

- DOI: [10.1109/TRO.2017.2659727](https://doi.org/10.1109/TRO.2017.2659727).
- Проверка: [CaltechAUTHORS](https://authors.library.caltech.edu/records/tshzw-g4v69).
- Авторская копия: [PDF](https://liwanggt.github.io/files/A2_Safe_Swarm.pdf).
- Статус: **университетская карточка/аннотация**, полный текст отдельно не анализировался.
- Использование: QP-коррекция номинального управления, включая децентрализованный вариант. Гарантии статьи нельзя автоматически переносить на произвольную дискретизацию и обрезание скоростей.

### R10 — современное непотенциальное поле

Yan Gao; Chenggang Bai; Rao Fu; Quan Quan. **A non-potential orthogonal vector field method for more efficient robot navigation and control**. 2023. *Robotics and Autonomous Systems*, 159, 104291.

- DOI: [10.1016/j.robot.2022.104291](https://doi.org/10.1016/j.robot.2022.104291).
- Проверка: [издатель, карточка и аннотация](https://www.sciencedirect.com/science/article/abs/pii/S0921889022001804).
- Статус: **карточка/аннотация издателя** найдена и прочитана через поиск; прямое открытие получило 403. Полный текст не проверен.
- Использование: модификация APF с ортогональным полем при определённых условиях, для одного робота и роя. Нельзя объявлять добавление непотенциального вклада новым принципом. Год статьи — 2023, несмотря на 2022 в DOI.

### R11 — современное переключение формации в узком проходе

Duy-Nam Bui; Manh Duong Phung; Hung Pham Duy. **Event-based Reconfiguration Control for Time-varying Formation of Robot Swarms in Narrow Spaces**. 2025. *Intelligent Service Robotics*.

- DOI журнальной работы: [10.1007/s11370-025-00607-z](https://doi.org/10.1007/s11370-025-00607-z).
- Первичная авторская версия: [arXiv:2505.16087v1](https://arxiv.org/abs/2505.16087v1), [HTML](https://arxiv.org/html/2505.16087v1).
- Статус: **текст авторской версии**; её карточка указывает журнал и связанный DOI. Окончательная журнальная верстка отдельно не сверена.
- Использование: APF-поведения, событийное переключение между формацией и следованием друг за другом. Это особенно близкий аналог предлагаемого направления ВКР; исходный код доступен по ссылке в статье, но здесь не запускался.

## Дополнительная современная работа

### R12 — адаптивная формация с MPC

Seung-Mok Lee. **Environment-adaptive Multi-robot Formation Planning and Control in Confined Spaces**. 2025. *International Journal of Control, Automation and Systems*, 23, 3126–3138. Опубликована 16 октября 2025.

- DOI: [10.1007/s12555-025-0473-z](https://doi.org/10.1007/s12555-025-0473-z).
- Проверка: [издатель](https://link.springer.com/article/10.1007/s12555-025-0473-z).
- Статус: **карточка/аннотация издателя**; полный текст платный и не прочитан.
- Использование: выбор конфигурации по локальной 2D occupancy grid и MPC. Валидация заявлена в Gazebo/ROS; это не означает обязательности ROS для математической идеи. Не выбран для первой реализации из-за объёма оптимизационного слоя; децентрализованность всех стадий по аннотации не установлена.

## Что осталось непроверенным

- Не проверены экспериментальным запуском показатели ни одной работы; опубликованные успехи не считаются результатами проекта пользователя.
- Не установлены исчерпывающий приоритет и отсутствие аналогов конкретной адаптивной эвристики ниже.
- **Кандидат для дальнейшего чтения, не опора содержательных выводов:** Vinicius Mariano Gonçalves, Prashanth Krishnamurthy, Anthony Tzes, Farshad Khorrami, *Control Barrier Functions With Circulation Inequalities*, 2024, предполагаемый DOI [10.1109/TCST.2024.3372802](https://doi.org/10.1109/TCST.2024.3372802), [IEEE document 10472718](https://ieeexplore.ieee.org/document/10472718/). Название найдено у издателя, но реквизиты доступны преимущественно в индексах; прямое открытие IEEE блокирует JavaScript. Полная первичная проверка авторов, DOI и текста в этом обзоре не завершена.

## Поисковый след

Использованы сочетания названий и запросов `artificial potential field Khatib`, `gyroscopic collision avoidance multiple agents`, `non-potential orthogonal vector field`, `Boids Reynolds`, `flocking Olfati-Saber`, `formation narrow passages reconfiguration 2023 2024 2025`, `velocity obstacles`, `ORCA`, `safety barrier certificates`. Проверка велась по издательским карточкам, авторским страницам, университетским репозиториям и arXiv. Поисковые индексы помогали найти первичные источники; пересказ по сторонним обзорным страницам не использовался как техническое основание. Полнота публикаций 2026 года не заявляется.
