import pygame

# 初始化 Pygame
pygame.init()

# 设置屏幕大小
screen = pygame.display.set_mode((640, 480))

# 设置运行标志，用于控制游戏循环
running = True

# 游戏循环
while running:
    # 处理事件
    for event in pygame.event.get():
        # 检查是否点击了关闭按钮
        if event.type == pygame.QUIT:
            running = False
        # 检查是否有按键被按下
        elif event.type == pygame.KEYDOWN:
            # 打印按键的键码
            print(f'键码: {event.key}')

# 退出 Pygame
pygame.quit()
