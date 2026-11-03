#pragma once
#include <QString>
#include <functional>

namespace demo
{
    int runSelfTest(const QString& reportPath);

    using TestCase = std::function<void(QString, const std::function<void()>&)>;
    void runCvSelfTests(const TestCase& test, const QString& reportPath);
} // namespace demo
